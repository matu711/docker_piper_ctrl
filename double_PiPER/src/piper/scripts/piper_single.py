"""Preserve single-arm v6 CSV/NPZ/JPEG/EE schema with the shared recorder UI."""
from types import SimpleNamespace
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from piper_msgs.msg import PiperEulerPose
from piper_recording_common import RawEpisodeWriter, extract_ee_pose, make_episode_name


def configure_metadata(metadata):
    metadata.update(format="piper_raw_fixed_rate_synced_v6_ee_pose",
        robot_type=rospy.get_param("~robot_type", "piper_single_arm"),
        observation_ee_source=rospy.get_param("~ee_topic", "/follower/end_pose"),
        ee_euler_topic=rospy.get_param("~ee_euler_topic", "/follower/end_pose_euler"),
        raw_leader_topic=rospy.get_param("~leader_topic", "/leader/joint_grip_pose"),
        ee_observation_dim=8, ee_position_unit="m",
        observation_ee_layout=["x_m", "y_m", "z_m", "qx", "qy", "qz", "qw", "gripper_m"],
        observation_ee_orientation="quaternion_xyzw",
        observation_ee_gripper_source=metadata["topics"]["single_state"],
        observation_state_source=metadata["topics"]["single_state"],
        action_source=metadata["topics"]["single_action"],
        value_names=metadata["names"],
        mcap_semantics="Original State/Action/EE/leader streams plus synchronized compressed images")

    if not metadata.get("cameras"):
        metadata["format"] = "piper_single_state_mcap_v1_ee_pose"


def add_sources(recorder):
    from piper_recorder import Buffer
    recorder.sources["ee"] = Buffer(recorder.metadata["observation_ee_source"], "ee", recorder.mcap,
                                    extractor=extract_ee_pose, msg_class=PoseStamped)
    recorder.mirrors = []
    for topic, msg_class in ((recorder.metadata["raw_leader_topic"], JointState),
                             (recorder.metadata["ee_euler_topic"], PiperEulerPose)):
        recorder.mirrors.append(rospy.Subscriber(topic, msg_class,
            lambda msg, topic=topic: recorder.mcap.write_ros_message(topic, msg), queue_size=200))


class SingleEpisode:
    """Translate shared synchronized samples into the unchanged legacy writer."""
    def __init__(self, root, metadata, anchor):
        self.metadata, self.anchor = dict(metadata), anchor
        self.rows = []
        self.dropped = {"missing_or_unsynced": 0, "schedule": 0}
        self.name = make_episode_name(root)
        cameras = metadata.get("camera_requested", {})
        front = cameras.get("front", {})
        wrist = cameras.get("wrist", {})
        self.args = SimpleNamespace(task=metadata["task"], fps=metadata["fps"], state_dim=7,
            follower_topic=metadata["topics"]["single_state"], action_topic=metadata["topics"]["single_action"],
            ee_topic=metadata["observation_ee_source"], leader_topic=metadata["raw_leader_topic"],
            sync_tolerance=metadata["sync_tolerance"], sync_wait=metadata["sync_wait"],
            camera_sync_tolerance=metadata.get("camera_sync_tolerance", .03),
            camera_sync_wait=metadata.get("camera_sync_wait", .1),
            camera_target_tolerance=metadata.get("camera_target_tolerance", .1),
            camera_fourcc=front.get("fourcc", "MJPG"), camera_fps=front.get("fps", 30),
            front_camera=front.get("device", ""), wrist_camera=wrist.get("device", ""),
            width=front.get("width", 640), height=front.get("height", 480),
            repo_id=metadata["repo_id"], robot_type=metadata["robot_type"])
        self.writer = RawEpisodeWriter(root, self.name, self.args, metadata["names"])
        self.path = self.writer.temp_dir
        self.csv = self.writer.csv_file
        self.frame_jpegs = {}
        self.last_sequences = {"front": None, "wrist": None}
        self.reused = {"front": 0, "wrist": 0}

    def stage_image(self, name, data):
        # RawEpisodeWriter owns the actual JPEG writes for the legacy schema.
        self.frame_jpegs[name] = data

    def add(self, target, samples, images=None, image_paths=None):
        state, state_stamp, state_error = samples["single_state"]
        action, action_stamp, action_error = samples["single_action"]
        ee, ee_stamp, ee_error = samples["ee"]
        pose = np.concatenate((ee, state[6:7]))
        stamps = [images[name][2] for name in ("front", "wrist")]
        payloads = [self.frame_jpegs[name] for name in ("front", "wrist")]
        self.writer.add_frame(payloads[0], payloads[1], state, action, pose,
            target, stamps[0], stamps[1], self.writer.start_monotonic + target-self.anchor,
            abs(stamps[0]-stamps[1]), state_stamp, action_stamp, ee_stamp,
            state_error, action_error, ee_error)
        self.frame_jpegs.clear()
        if images:
            for name in self.last_sequences:
                self.reused[name] += images[name][0] == self.last_sequences[name]
                self.last_sequences[name] = images[name][0]
        self.rows.append(target)

    def finish(self, status, reason=""):
        if status == "saved":
            self.writer.dropped_sync_frames = self.dropped["missing_or_unsynced"]
            self.writer.dropped_schedule_frames = self.dropped["schedule"]
            for name in ("front", "wrist"):
                setattr(self.writer, "reused_"+name+"_frames", self.reused[name])
                setattr(self.writer, name+"_capture_fps", self.metadata.get("camera_measured_fps", {}).get(name, 0.))
            self.path = self.writer.save()
        else:
            self.writer.leave_for_recovery(reason)
        return self.path
