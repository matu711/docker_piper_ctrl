#!/usr/bin/env python3
"""Three-camera, 14-dimensional extension of the single-arm MCAP recorder."""
import argparse
import csv
import itertools
import json
import os
import queue
import shutil
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

from record_piper_mcap import (McapEpisodeRecorder, McapRos1Writer, RosVectorBuffer,
                              make_compressed_image)
from mcap2_camera import CameraCapture

CAMERAS = ("front", "wrist1", "wrist2")
STREAMS = ("right_state", "right_action", "left_state", "left_action")
VALUE_NAMES = [side + "_" + joint for side in ("right", "left")
               for joint in ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper")]


def select_images(snapshots, target, skew_limit, age_limit):
    candidates = [[item for item in frames if abs(item[2]-target) <= age_limit] for frames in snapshots]
    choices = [items for items in itertools.product(*candidates)
               if max(item[2] for item in items)-min(item[2] for item in items) <= skew_limit]
    return min(choices, key=lambda items: max(abs(item[2]-target) for item in items)) if choices else None


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path.home()/"piper_dataset"/"dual2")
    parser.add_argument("--repo-id", default="local/piper_dual2")
    parser.add_argument("--task", default="bimanual manipulation")
    for name, value in (("fps", 20), ("camera-fps", 30), ("width", 640), ("height", 480), ("jpeg-quality", 95)):
        parser.add_argument("--"+name, type=int, default=value)
    for name, value in (("sync-tolerance", .03), ("sync-wait", .025), ("camera-sync-tolerance", .03),
                        ("camera-target-tolerance", .1), ("max-data-gap", 1.), ("camera-open-timeout", 5.),
                        ("camera-read-timeout", 2.), ("control-timeout", 20.), ("sync-delay", .2)):
        parser.add_argument("--"+name, type=float, default=value)
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--backend", choices=("v4l2", "default"), default="v4l2")
    def boolean(value):
        if value.lower() not in ("true", "false"):
            raise argparse.ArgumentTypeError("Expected true or false")
        return value.lower() == "true"
    parser.add_argument("--no-preview", nargs="?", const=True, default=False, type=boolean)
    for name, device in zip(CAMERAS, ("/dev/video4", "/dev/video2", "/dev/video0")):
        parser.add_argument("--"+name+"-camera", default=device)
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    if min(args.fps, args.camera_fps, args.width, args.height, args.sync_tolerance,
           args.camera_sync_tolerance, args.camera_target_tolerance, args.max_data_gap, args.camera_open_timeout,
           args.camera_read_timeout, args.control_timeout) <= 0:
        raise ValueError("Rates, sizes and timeouts must be positive")
    if min(args.sync_wait, args.sync_delay) < 0 or not 1 <= args.jpeg_quality <= 100 or len(args.fourcc) != 4:
        raise ValueError("Invalid sync wait, JPEG quality or FourCC")
    if len({getattr(args, name+"_camera") for name in CAMERAS}) != 3:
        raise ValueError("Three distinct camera devices required")
    return args


class Episode:
    def __init__(self, args):
        self.args = args
        cv2.setNumThreads(1)
        self.name = "episode_" + str(time.time_ns())
        self.path = args.root/(".tmp_"+self.name)
        self.path.mkdir()
        for name in CAMERAS:
            (self.path/name).mkdir()
        self.rows = []
        self.finished = False
        self.dropped = {"missing_or_unsynced": 0, "schedule": 0}
        self.previous = dict.fromkeys(CAMERAS, None)
        self.reused = dict.fromkeys(CAMERAS, 0)
        self.columns = ["frame_index", "timestamp", "unix_timestamp", "image_ros_timestamp", "camera_time_error"]
        self.columns += [key+suffix for key in STREAMS+CAMERAS for suffix in ("_ros_timestamp", "_time_error")]
        self.columns += [name+"_image" for name in CAMERAS]
        self.columns += [kind+"_"+name for kind in ("state", "action") for name in VALUE_NAMES]
        self.file = (self.path/"frames.partial.csv").open("w", newline="")
        self.csv = csv.DictWriter(self.file, fieldnames=self.columns)
        self.csv.writeheader()
        self.metadata("recording")

    def metadata(self, status, error=None):
        data = {"format": "piper_dual_mcap2_v1", "status": status, "error": error,
                "state_dim": 14, "action_dim": 14, "value_names": VALUE_NAMES,
                "arm_order": ["right", "left"], "camera_names": CAMERAS,
                "units": {"joints": "rad", "gripper": "m", "timestamps": "s"},
                "fps": self.args.fps, "camera_fps": self.args.camera_fps,
                "repo_id": self.args.repo_id, "task": self.args.task,
                "num_frames": len(self.rows), "dropped_frames": self.dropped, "reused_images": self.reused,
                "sync_tolerance": self.args.sync_tolerance, "camera_sync_tolerance": self.args.camera_sync_tolerance,
                "sync_delay": getattr(self.args, "sync_delay", 0.),
                "camera_target_tolerance": self.args.camera_target_tolerance,
                "capture_settings": getattr(self.args, "capture_settings", {}),
                "measured_camera_fps": getattr(self.args, "measured_camera_fps", {}),
                "timestamp_source": "SDK receive timestamps; post-SDK action timestamp; V4L2 timestamp or validated read-midpoint fallback",
                "action_semantics": "SDK-dispatched command, not a hardware acknowledgement"}
        temporary = self.path/"meta.json.tmp"
        temporary.write_text(json.dumps(data, indent=2))
        os.replace(temporary, self.path/"meta.json")

    def add(self, index, target, anchor, streams, images, jpegs):
        row = {"frame_index": index, "timestamp": target-anchor, "unix_timestamp": target,
               "image_ros_timestamp": target,
               "camera_time_error": max(frame[2] for frame in images)-min(frame[2] for frame in images)}
        for name, (_, stamp, error) in streams.items():
            row[name+"_ros_timestamp"], row[name+"_time_error"] = stamp, error
        for name, frame, jpeg in zip(CAMERAS, images, jpegs):
            relative = "%s/%06d.jpg" % (name, index)
            (self.path/relative).write_bytes(jpeg)
            row[name+"_image"] = relative
            row[name+"_ros_timestamp"], row[name+"_time_error"] = frame[2], frame[2]-target
            if self.previous[name] == frame[0]:
                self.reused[name] += 1
            self.previous[name] = frame[0]
        for kind in ("state", "action"):
            vector = np.concatenate((streams["right_"+kind][0], streams["left_"+kind][0]))
            row.update(zip([kind+"_"+name for name in VALUE_NAMES], vector.tolist()))
        self.rows.append(row)
        self.csv.writerow(row)
        self.file.flush()

    def finish(self, status="saved", error=None):
        if self.finished:
            return self.path
        self.file.flush()
        os.fsync(self.file.fileno())
        self.file.close()
        arrays = {key: np.asarray([row[key] for row in self.rows]) for key in self.columns
                  if not key.endswith("_image") and not key.startswith(("state_", "action_"))}
        for kind, key in (("state", "observation_state"), ("action", "action")):
            arrays[key] = np.asarray([[row[kind+"_"+name] for name in VALUE_NAMES] for row in self.rows],
                                     dtype=np.float32).reshape(-1, 14)
        temporary = self.path/"data.npz.tmp"
        with temporary.open("wb") as stream:
            np.savez_compressed(stream, **arrays)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.path/"data.npz")
        (self.path/"frames.partial.csv").rename(self.path/"frames.csv")
        self.metadata(status, error)
        if status == "saved":
            final = self.args.root/self.name
            self.path.rename(final)
            self.path = final
        self.finished = True
        return self.path


class Recorder:
    def __init__(self, args):
        self.args = args
        if McapRos1Writer is None:
            raise RuntimeError("Install mcap and mcap-ros1-support in this Python environment")
        args.root = args.root.expanduser().resolve()
        args.root.mkdir(parents=True, exist_ok=True)
        schema = {"format": "piper_dual_mcap2_v1", "value_names": VALUE_NAMES, "camera_names": list(CAMERAS), "fps": args.fps}
        meta = args.root/"dataset_meta.json"
        if meta.exists() and json.loads(meta.read_text()) != schema:
            raise RuntimeError("Dataset schema differs; use a new dataset_root")
        meta.write_text(json.dumps(schema, indent=2))
        self.lock = threading.RLock()
        self.operation_lock = threading.Lock()
        self.status_lock = threading.Lock()
        self.closing = threading.Event()
        self.shutdown_lock = threading.Lock()
        self.closed = False
        self.camera_capture = None
        rospy.on_shutdown(self.closing.set)
        self.status, self.status_at = {}, 0.
        self.episode = None
        self.mcap = McapEpisodeRecorder()
        self.sources, self.cameras, self.image_pubs = {}, {}, {}
        self.last_sequence = dict.fromkeys(CAMERAS, -1)
        self.status_sub = rospy.Subscriber("/dual_piper/status", String, self.status_callback, queue_size=1)
        for side, suffix in (("right", "R"), ("left", "L")):
            for kind, leaf in (("state", "current_joint_obs"), ("action", "applied_joint_cmd")):
                key = side+"_"+kind
                self.sources[key] = RosVectorBuffer("/follower"+suffix+"/"+leaf, key, 500, self.mcap)
        # Known JointState type avoids blocking discovery of raw leader topics.
        self.leader_subs = [rospy.Subscriber("/leader"+suffix+"/joint_grip_pose", JointState,
            lambda msg, topic: self.mcap.write_ros_message(topic, msg),
            callback_args="/leader"+suffix+"/joint_grip_pose", queue_size=10) for suffix in ("R", "L")]
        if rospy.get_param("/use_sim_time", False):
            raise RuntimeError("Real cameras require /use_sim_time=false")
        try:
            devices = [(name, getattr(args, name+"_camera")) for name in CAMERAS]
            config = {"backend": args.backend, "width": args.width, "height": args.height,
                      "fps": args.camera_fps, "fourcc": args.fourcc,
                      "open_timeout": args.camera_open_timeout, "read_timeout": args.camera_read_timeout}
            self.camera_capture = CameraCapture(devices, config)
            self.camera_capture.wait_ready((args.camera_open_timeout+10)*3+2,
                                          lambda: self.closing.is_set() or rospy.is_shutdown())
            args.capture_settings = self.camera_capture.settings
            self.cameras = self.camera_capture.views
            from sensor_msgs.msg import CompressedImage
            for name, settings in args.capture_settings.items():
                rospy.loginfo("Camera %s: %dx%d @ %.2f FPS FourCC=%s; output=%dx%d", settings["device"],
                    settings["width"], settings["height"], settings["fps"], settings["fourcc"], args.width, args.height)
                self.image_pubs[name] = rospy.Publisher("/camera/"+name+"/compressed", CompressedImage, queue_size=1)
        except Exception:
            self.shutdown()
            raise
        for name in ("start", "stop", "discard"):
            rospy.Service("/dual_recorder/"+name, Trigger, self.service(name))
        print("r=teach/follow/record t=teaching mode c=resume following s=save v=save x=discard p=save poses h=return "
              "u=enable e=recover follower STOP [confirm safety] space=hold q=quit")

    def status_callback(self, msg):
        with self.status_lock:
            self.status, self.status_at = json.loads(msg.data), time.monotonic()
        self.mcap.write_ros_message("/dual_piper/status", msg)

    def healthy(self):
        with self.status_lock:
            if time.monotonic()-self.status_at > .5 or self.status.get("fault") or not self.status.get("following"):
                raise RuntimeError("Both followers must be healthy and following; check controller status")

    def control(self, name, timeout=None):
        if self.closing.is_set() and name != "stop_following":
            raise RuntimeError("Recorder is shutting down")
        path = "/dual_piper/"+name
        result_queue = queue.Queue(maxsize=1)
        def call_service():
            try:
                rospy.wait_for_service(path, timeout=2.)
                result_queue.put((True, rospy.ServiceProxy(path, Trigger)()))
            except Exception as exc:
                result_queue.put((False, exc))
        threading.Thread(target=call_service, daemon=True).start()
        try:
            ok, result = result_queue.get(timeout=timeout if timeout is not None else self.args.control_timeout)
        except queue.Empty:
            raise RuntimeError("Control service timed out: "+path)
        if not ok:
            raise RuntimeError(str(result))
        if not result.success:
            raise RuntimeError(result.message)
        return result.message

    def service(self, name):
        def invoke(_):
            if not self.operation_lock.acquire(False):
                return TriggerResponse(False, "Another recorder operation is active")
            try:
                return TriggerResponse(True, getattr(self, name)())
            except Exception as exc:
                return TriggerResponse(False, str(exc))
            finally:
                self.operation_lock.release()
        return invoke

    def start(self):
        with self.lock:
            if self.episode:
                raise RuntimeError("Already recording")
        try:
            # Match single-arm prepare_recording_async: teach -> resume -> record.
            self.control("enter_teaching_mode")
            self.control("start_following")
            deadline = time.monotonic()+2.
            while True:
                try:
                    self.healthy()
                    target = rospy.Time.now().to_sec()
                    if any(v is None or v.size != 7 or not np.isfinite(v).all() or abs(e)>self.args.sync_tolerance
                           for v, _, e in (source.nearest(target) for source in self.sources.values())):
                        raise RuntimeError("Waiting for fresh State/Action")
                    break
                except RuntimeError:
                    if time.monotonic() >= deadline or rospy.is_shutdown():
                        raise
                    time.sleep(.02)
            with self.lock:
                if self.closing.is_set():
                    raise RuntimeError("Recorder is shutting down")
                episode = Episode(self.args)
                try:
                    self.mcap.start(episode.path)
                except Exception as exc:
                    episode.finish("incomplete", str(exc))
                    raise
                self.episode = episode
                self.anchor = self.last_ros = rospy.Time.now().to_sec()
                self.tick_index = 0
                self.last_saved = time.monotonic()
            return "Recording "+episode.name
        except Exception:
            self.control("stop_following")
            raise

    def stop(self):
        self.control("stop_following")
        with self.lock:
            if not self.episode:
                raise RuntimeError("Not recording")
            self.mcap.finish()
            self.args.measured_camera_fps = {name: view.measured_fps() for name, view in self.cameras.items()}
            status = "saved" if self.episode.rows else "incomplete"
            path = self.episode.finish(status)
            self.episode = None
        return "Saved "+str(path)+"; h to return followers to initial poses"

    def discard(self):
        self.control("stop_following")
        with self.lock:
            if not self.episode:
                raise RuntimeError("Not recording")
            self.mcap.discard()
            self.episode.file.close()
            shutil.rmtree(self.episode.path)
            self.episode = None
        return "Discarded; following stopped"

    def abort(self, error):
        with self.lock:
            if self.episode:
                self.mcap.finish_incomplete()
                self.args.measured_camera_fps = {name: view.measured_fps() for name, view in getattr(self, "cameras", {}).items()}
                self.episode.finish("incomplete", str(error))
                self.episode = None
        try:
            self.control("stop_following", timeout=.5 if self.closing.is_set() else 2.)
        except Exception as exc:
            rospy.logerr("Could not stop following: %s", exc)

    def shutdown(self):
        # Do not block the ROS signal handler on capture or service locks.
        self.closing.set()
        with self.shutdown_lock:
            if self.closed:
                return
            self.closed = True
            try:
                # Finalize MCAP/CSV/NPZ before touching possibly stuck capture.
                self.abort("Recorder shutdown before save")
            finally:
                if self.camera_capture is not None:
                    forced = self.camera_capture.close()
                    if forced:
                        rospy.logwarn("Stopped unresponsive camera subprocess; recorder files were finalized")
                if not self.args.no_preview:
                    cv2.destroyAllWindows()

    def step(self):
        snapshots = [self.cameras[name].frame_snapshot() for name in CAMERAS]
        now = rospy.Time.now().to_sec()
        if any(not frames or time.monotonic()-frames[-1][3]>self.args.camera_read_timeout for frames in snapshots):
            raise RuntimeError("Camera feedback missing/stale")
        with self.lock:
            if self.episode:
                self.healthy()
                if now < self.last_ros or now-self.last_ros > self.args.max_data_gap:
                    raise RuntimeError("ROS time discontinuity")
                self.last_ros = now
                # Camera timestamps describe an already captured frame. Wait
                # briefly for its USB/decode/IPC delivery, then match historical
                # state/action to that grid time; never shift the timestamps.
                due = int((now-self.anchor-self.args.sync_delay)*self.args.fps)
                if due > self.tick_index+1:
                    self.episode.dropped["schedule"] += due-self.tick_index-1
                    self.tick_index = due-1
                target = self.anchor+self.tick_index/self.args.fps
                if now >= target+self.args.sync_delay:
                    deadline = time.monotonic()+self.args.sync_wait
                    for source in self.sources.values():
                        source.wait_until_at_least(target, deadline)
                    streams = {name: source.nearest(target) for name, source in self.sources.items()}
                    images = select_images(snapshots, target, self.args.camera_sync_tolerance, self.args.camera_target_tolerance)
                    valid = images is not None and all(v is not None and v.size == 7 and np.isfinite(v).all()
                        and abs(error)<=self.args.sync_tolerance for v, _, error in streams.values())
                    if valid:
                        compressed = [make_compressed_image(frame[1], rospy.Time.from_sec(frame[2]), name,
                                      self.args.jpeg_quality) for name, frame in zip(CAMERAS, images)]
                        self.episode.add(self.tick_index, target, self.anchor, streams, images,
                                         [jpeg for _, jpeg in compressed])
                        self.last_saved = time.monotonic()
                        for name, (msg, _) in zip(CAMERAS, compressed):
                            self.mcap.write_ros_message("/camera/"+name+"/compressed", msg)
                    else:
                        self.episode.dropped["missing_or_unsynced"] += 1
                    self.tick_index += 1
                if time.monotonic()-self.last_saved > self.args.max_data_gap:
                    raise RuntimeError("No synchronized data within max-data-gap")
        preview = []
        for name, frames in zip(CAMERAS, snapshots):
            sequence, frame, stamp, _ = frames[-1]
            if sequence != self.last_sequence[name]:
                msg, _ = make_compressed_image(frame, rospy.Time.from_sec(stamp), name, self.args.jpeg_quality)
                self.image_pubs[name].publish(msg)
                self.last_sequence[name] = sequence
            if not self.args.no_preview:
                shown = frame.copy()
                cv2.putText(shown, name, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 255, 255), 2)
                preview.append(shown)
        if preview:
            cv2.putText(preview[0], "RECORDING" if self.episode else "IDLE", (12, 64),
                        cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 255, 255), 2)
            cv2.imshow("PiPER MCAP2: front / wrist1 / wrist2", np.concatenate(preview, axis=1))
            return cv2.waitKey(1) & 255
        return -1

    def operate(self, key):
        operations = {ord("r"): "start", ord("s"): "stop", ord("v"): "stop", ord("x"): "discard"}
        if key in operations:
            result = self.service(operations[key])(None)
            (rospy.loginfo if result.success else rospy.logerr)(result.message)
            return
        controls = {ord("c"): "start_following", ord("t"): "enter_teaching_mode", ord(" "): "stop_following", ord("p"): "save_initial_pose",
                    ord("h"): "go_initial_pose", ord("u"): "enable", ord("e"): "recover_stop"}
        if key in controls:
            if not self.operation_lock.acquire(False):
                return
            try:
                with self.lock:
                    if self.episode:
                        raise RuntimeError("Save/discard recording before changing control state")
                rospy.loginfo(self.control(controls[key]))
            except Exception as exc:
                rospy.logerr(str(exc))
            finally:
                self.operation_lock.release()

    def run(self):
        try:
            while not rospy.is_shutdown() and not self.closing.is_set():
                try:
                    key = self.step()
                except Exception as exc:
                    self.abort(exc)
                    rospy.logerr_throttle(2., str(exc))
                    key = -1
                if key == ord("q"):
                    break
                if key in map(ord, "rsvxp hcuet"):
                    threading.Thread(target=self.operate, args=(key,), daemon=True).start()
                time.sleep(.005)
        finally:
            self.shutdown()


if __name__ == "__main__":
    rospy.init_node("record_dual_piper_mcap2")
    Recorder(parse_args()).run()
