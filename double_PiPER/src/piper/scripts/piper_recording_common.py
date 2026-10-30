"""Shared MCAP, timestamp buffers and compatible single-arm writer.

No CLI, camera ownership, robot services, or collection loop lives here.
"""

import argparse


import copy


import csv


import json


import numbers


import shutil




import threading


import time


from collections import deque


from datetime import datetime


from pathlib import Path


from typing import Any, Deque, List, Optional, Tuple


import cv2


import numpy as np


import rospy


import rostopic


from sensor_msgs.msg import CompressedImage




try:
    from mcap_ros1.writer import Writer as McapRos1Writer
except ImportError:
    McapRos1Writer = None


DEFAULT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
    "gripper",
]


def is_numeric_scalar(value: Any) -> bool:
    return (
        isinstance(value, (numbers.Real, np.number))
        and not isinstance(value, bool)
    )


def flatten_numeric(
    value: Any,
    name: str = "",
) -> List[float]:
    if name.lower() in {
        "header",
        "stamp",
        "seq",
        "frame_id",
        "timestamp",
        "time",
    }:
        return []

    if is_numeric_scalar(value):
        return [float(value)]

    if isinstance(value, np.ndarray):
        if np.issubdtype(value.dtype, np.number):
            return (
                value.reshape(-1)
                .astype(np.float64)
                .tolist()
            )
        return []

    if isinstance(value, (list, tuple)):
        result = []
        for item in value:
            result.extend(flatten_numeric(item))
        return result

    slots = getattr(value, "__slots__", None)
    if slots:
        result = []
        for slot in slots:
            if hasattr(value, slot):
                result.extend(
                    flatten_numeric(
                        getattr(value, slot),
                        name=slot,
                    )
                )
        return result

    return []


def extract_vector(msg: Any) -> np.ndarray:
    if hasattr(msg, "position"):
        try:
            vector = np.asarray(
                msg.position,
                dtype=np.float32,
            ).reshape(-1)
            if vector.size:
                return vector
        except (TypeError, ValueError):
            pass

    if hasattr(msg, "data") and not isinstance(
        msg.data,
        (str, bytes),
    ):
        try:
            vector = np.asarray(
                msg.data,
                dtype=np.float32,
            ).reshape(-1)
            if vector.size:
                return vector
        except (TypeError, ValueError):
            pass

    vector = np.asarray(
        flatten_numeric(msg),
        dtype=np.float32,
    ).reshape(-1)

    if vector.size:
        return vector

    raise ValueError(
        "Could not extract numeric vector from {}".format(
            type(msg).__name__
        )
    )


def extract_ee_pose(msg: Any) -> np.ndarray:
    """Return PoseStamped as exactly XYZ[m] + quaternion XYZW."""
    pose = getattr(msg, "pose", None)
    position = getattr(pose, "position", None)
    orientation = getattr(pose, "orientation", None)
    values = [
        getattr(position, "x", None), getattr(position, "y", None),
        getattr(position, "z", None), getattr(orientation, "x", None),
        getattr(orientation, "y", None), getattr(orientation, "z", None),
        getattr(orientation, "w", None),
    ]
    if any(value is None for value in values):
        raise ValueError("expected geometry_msgs/PoseStamped")
    vector = np.asarray(values, dtype=np.float32)
    if not np.all(np.isfinite(vector)):
        raise ValueError("end-effector pose contains non-finite values")
    return vector


def get_ros_stamp(msg: Any) -> float:
    try:
        stamp = float(msg.header.stamp.to_sec())
        if stamp > 0.0:
            return stamp
    except Exception:
        pass

    # Fallback only for messages without a valid header stamp.
    return float(rospy.Time.now().to_sec())


def select_vector(
    raw: np.ndarray,
    start: int,
    dim: int,
    label: str,
) -> np.ndarray:
    end = start + dim

    if raw.size < end:
        raise ValueError(
            "{} has {} values; requested [{}:{}]".format(
                label,
                raw.size,
                start,
                end,
            )
        )

    return raw[start:end].astype(
        np.float32,
        copy=False,
    )


class McapEpisodeRecorder:
    def __init__(self):
        self.lock = threading.Lock()
        self.writer = None
        self.file = None
        self.partial_path = None
        self.final_path = None

    @property
    def active(self) -> bool:
        return self.writer is not None

    def start(self, episode_dir: Path) -> None:
        if McapRos1Writer is None:
            raise RuntimeError(
                "MCAP support is not installed.\n"
                "Run: python3 -m pip install mcap mcap-ros1-support"
            )

        with self.lock:
            if self.writer is not None:
                raise RuntimeError(
                    "MCAP recorder is already active"
                )

            episode_name = episode_dir.name
            if episode_name.startswith(".tmp_"):
                episode_name = episode_name[5:]

            self.partial_path = (
                episode_dir
                / (episode_name + ".mcap.partial")
            )
            self.final_path = (
                episode_dir
                / (episode_name + ".mcap")
            )

            self.file = self.partial_path.open("wb")
            self.writer = McapRos1Writer(self.file)

    @staticmethod
    def _message_time_ns(msg: Any) -> int:
        try:
            stamp_ns = int(msg.header.stamp.to_nsec())
            if stamp_ns > 0:
                return stamp_ns
        except Exception:
            pass

        return time.time_ns()

    def write_ros_message(
        self,
        topic: str,
        msg: Any,
    ) -> None:
        with self.lock:
            if self.writer is None:
                return

            timestamp_ns = self._message_time_ns(msg)

            self.writer.write_message(
                topic,
                msg,
                log_time=timestamp_ns,
                publish_time=timestamp_ns,
            )

    def finish(self) -> Optional[Path]:
        with self.lock:
            if self.writer is None:
                return None

            self.writer.finish()
            self.file.close()

            partial_path = self.partial_path
            final_path = self.final_path

            self.writer = None
            self.file = None
            self.partial_path = None
            self.final_path = None

        partial_path.rename(final_path)
        return final_path

    def discard(self) -> None:
        with self.lock:
            if self.writer is None:
                return

            try:
                self.writer.finish()
            except Exception:
                pass

            try:
                self.file.close()
            except Exception:
                pass

            partial_path = self.partial_path

            self.writer = None
            self.file = None
            self.partial_path = None
            self.final_path = None

        if (
            partial_path is not None
            and partial_path.exists()
        ):
            partial_path.unlink()

    def finish_incomplete(self) -> Optional[Path]:
        with self.lock:
            if self.writer is None:
                return None

            try:
                self.writer.finish()
            finally:
                try:
                    self.file.close()
                except Exception:
                    pass

            partial_path = self.partial_path
            incomplete_path = partial_path.with_name(
                partial_path.name.replace(
                    ".mcap.partial",
                    ".incomplete.mcap",
                )
            )

            self.writer = None
            self.file = None
            self.partial_path = None
            self.final_path = None

        if partial_path.exists():
            partial_path.rename(incomplete_path)

        return incomplete_path


class RosVectorBuffer:
    """
    Keep timestamped ROS vectors and select the sample nearest
    to a requested ROS timestamp.
    """

    def __init__(
        self,
        topic: str,
        label: str,
        maxlen: int,
        mcap_recorder: McapEpisodeRecorder,
        extractor=extract_vector,
    ):
        self.topic = topic
        self.label = label
        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)
        self.buffer = deque(maxlen=maxlen)
        self.error_reported = False
        self.mcap_recorder = mcap_recorder
        self.extractor = extractor

        msg_class, real_topic, _ = rostopic.get_topic_class(
            topic,
            blocking=True,
        )

        if msg_class is None or real_topic is None:
            raise RuntimeError(
                "ROS topic not found: {}".format(topic)
            )

        self.real_topic = real_topic
        self.msg_class = msg_class

        self.subscriber = rospy.Subscriber(
            real_topic,
            msg_class,
            self.callback,
            queue_size=10,
        )

        rospy.loginfo(
            "%s topic: %s [%s]",
            self.label,
            self.real_topic,
            self.msg_class._type,
        )

    def callback(self, msg: Any) -> None:
        try:
            vector = self.extractor(msg)
            stamp = get_ros_stamp(msg)

            with self.condition:
                self.buffer.append(
                    (
                        stamp,
                        vector.astype(
                            np.float32,
                            copy=True,
                        ),
                    )
                )
                self.condition.notify_all()

            # Store the original ROS message in MCAP while active.
            self.mcap_recorder.write_ros_message(
                self.real_topic,
                copy.deepcopy(msg),
            )

        except Exception as exc:
            if not self.error_reported:
                rospy.logerr(
                    "Failed to parse %s topic %s: %s",
                    self.label,
                    self.real_topic,
                    exc,
                )
                self.error_reported = True

    def has_data(self) -> bool:
        with self.lock:
            return bool(self.buffer)

    def wait_until_at_least(
        self,
        target_stamp: float,
        deadline_monotonic: float,
    ) -> None:
        """
        Wait briefly for a sample at/after target time.
        The same global deadline can be shared by state and action,
        so total waiting does not double.
        """
        with self.condition:
            while True:
                if (
                    self.buffer
                    and self.buffer[-1][0] >= target_stamp
                ):
                    return

                remaining = (
                    deadline_monotonic
                    - time.monotonic()
                )

                if remaining <= 0.0:
                    return

                self.condition.wait(
                    timeout=remaining
                )

    def nearest(
        self,
        target_stamp: float,
    ) -> Tuple[
        Optional[np.ndarray],
        float,
        float,
    ]:
        """
        Returns:
            vector,
            ros_timestamp,
            signed_error_seconds = sample_time - target_time
        """
        with self.lock:
            if not self.buffer:
                return None, float("nan"), float("inf")

            stamp, vector = min(
                self.buffer,
                key=lambda item: abs(
                    item[0] - target_stamp
                ),
            )

            return (
                vector.copy(),
                float(stamp),
                float(stamp - target_stamp),
            )


def make_compressed_image(
    frame_bgr: np.ndarray,
    stamp: rospy.Time,
    frame_id: str,
    jpeg_quality: int,
) -> Tuple[CompressedImage, bytes]:
    options = [
        int(cv2.IMWRITE_JPEG_QUALITY),
        int(jpeg_quality),
    ]

    ok, encoded = cv2.imencode(
        ".jpg",
        frame_bgr,
        options,
    )

    if not ok:
        raise RuntimeError(
            "Failed to JPEG encode {}".format(
                frame_id
            )
        )

    jpeg_data = encoded.tobytes()

    msg = CompressedImage()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.format = "jpeg"
    msg.data = jpeg_data

    return msg, jpeg_data


def make_episode_name(root: Path) -> str:
    base = datetime.now().strftime(
        "%Y%m%d_%H%M%S"
    )
    name = base
    suffix = 1

    while (
        (root / name).exists()
        or (root / (".tmp_" + name)).exists()
    ):
        name = "{}_{:02d}".format(
            base,
            suffix,
        )
        suffix += 1

    return name


def write_json(
    path: Path,
    value: Any,
) -> None:
    with path.open(
        "w",
        encoding="utf-8",
    ) as file:
        json.dump(
            value,
            file,
            ensure_ascii=False,
            indent=2,
        )


def make_names(
    state_dim: int,
) -> List[str]:
    if state_dim == len(DEFAULT_NAMES):
        return list(DEFAULT_NAMES)

    return [
        "value_{}".format(index)
        for index in range(state_dim)
    ]


class RawEpisodeWriter:

    def __init__(
        self,
        root: Path,
        episode_name: str,
        args: argparse.Namespace,
        value_names: List[str],
    ):
        self.root = root
        self.episode_name = episode_name
        self.args = args
        self.value_names = value_names

        self.temp_dir = root / (
            ".tmp_" + episode_name
        )
        self.final_dir = root / episode_name

        if self.temp_dir.exists():
            raise FileExistsError(
                "Temporary episode exists: {}".format(
                    self.temp_dir
                )
            )

        if self.final_dir.exists():
            raise FileExistsError(
                "Episode exists: {}".format(
                    self.final_dir
                )
            )

        self.front_dir = (
            self.temp_dir / "front"
        )
        self.wrist_dir = (
            self.temp_dir / "wrist"
        )

        self.front_dir.mkdir(parents=True)
        self.wrist_dir.mkdir(parents=True)

        self.csv_path = (
            self.temp_dir / "frames.partial.csv"
        )
        self.csv_file = self.csv_path.open(
            "w",
            newline="",
            encoding="utf-8",
        )
        self.csv_writer = csv.writer(
            self.csv_file
        )

        header = [
            "frame_index",
            "timestamp",
            "unix_timestamp",
            "image_ros_timestamp",
            "front_ros_timestamp",
            "wrist_ros_timestamp",
            "camera_time_error",
            "state_ros_timestamp",
            "action_ros_timestamp",
            "ee_ros_timestamp",
            "state_time_error",
            "action_time_error",
            "ee_time_error",
            "front_image",
            "wrist_image",
        ]

        header += [
            "state_{}".format(name)
            for name in value_names
        ]

        header += [
            "action_{}".format(name)
            for name in value_names
        ]
        header += [
            "ee_x_m", "ee_y_m", "ee_z_m",
            "ee_qx", "ee_qy", "ee_qz", "ee_qw", "ee_gripper_m",
        ]

        self.csv_writer.writerow(header)
        self.csv_file.flush()

        self.timestamp = []
        self.unix_timestamp = []
        self.image_ros_timestamp = []
        self.front_ros_timestamp = []
        self.wrist_ros_timestamp = []
        self.camera_time_error = []
        self.state_ros_timestamp = []
        self.action_ros_timestamp = []
        self.ee_ros_timestamp = []
        self.state_time_error = []
        self.action_time_error = []
        self.ee_time_error = []
        self.observation_state = []
        self.action = []
        self.observation_ee = []

        self.start_monotonic = time.monotonic()
        self.start_unix = time.time()
        self.num_frames = 0
        self.closed = False

        write_json(
            self.temp_dir
            / "recording_status.json",
            {
                "status": "recording",
                "episode_name": self.episode_name,
                "task": args.task,
                "fps": args.fps,
                "started_unix_timestamp": (
                    self.start_unix
                ),
            },
        )

    def add_frame(
        self,
        front_jpeg: bytes,
        wrist_jpeg: bytes,
        follower: np.ndarray,
        action: np.ndarray,
        ee: np.ndarray,
        image_ros_timestamp: float,
        front_ros_timestamp: float,
        wrist_ros_timestamp: float,
        image_monotonic_timestamp: float,
        camera_time_error: float,
        state_ros_timestamp: float,
        action_ros_timestamp: float,
        ee_ros_timestamp: float,
        state_time_error: float,
        action_time_error: float,
        ee_time_error: float,
    ) -> None:
        frame_index = self.num_frames

        front_name = "{:06d}.jpg".format(
            frame_index
        )
        wrist_name = "{:06d}.jpg".format(
            frame_index
        )

        (
            self.front_dir / front_name
        ).write_bytes(front_jpeg)

        (
            self.wrist_dir / wrist_name
        ).write_bytes(wrist_jpeg)

        relative_timestamp = (
            image_monotonic_timestamp
            - self.start_monotonic
        )
        now_unix = image_ros_timestamp

        follower_copy = follower.astype(
            np.float32,
            copy=True,
        )
        action_copy = action.astype(
            np.float32,
            copy=True,
        )
        ee_copy = ee.astype(np.float32, copy=True)
        if ee_copy.shape != (8,):
            raise ValueError("EE observation must have 8 values, got {}".format(ee_copy.shape))

        self.timestamp.append(
            relative_timestamp
        )
        self.unix_timestamp.append(
            now_unix
        )
        self.image_ros_timestamp.append(
            image_ros_timestamp
        )
        self.front_ros_timestamp.append(
            front_ros_timestamp
        )
        self.wrist_ros_timestamp.append(
            wrist_ros_timestamp
        )
        self.camera_time_error.append(
            camera_time_error
        )
        self.state_ros_timestamp.append(
            state_ros_timestamp
        )
        self.action_ros_timestamp.append(
            action_ros_timestamp
        )
        self.ee_ros_timestamp.append(ee_ros_timestamp)
        self.state_time_error.append(
            state_time_error
        )
        self.action_time_error.append(
            action_time_error
        )
        self.ee_time_error.append(ee_time_error)
        self.observation_state.append(
            follower_copy
        )
        self.action.append(
            action_copy
        )
        self.observation_ee.append(ee_copy)

        row = [
            frame_index,
            "{:.9f}".format(
                relative_timestamp
            ),
            "{:.9f}".format(now_unix),
            "{:.9f}".format(
                image_ros_timestamp
            ),
            "{:.9f}".format(
                front_ros_timestamp
            ),
            "{:.9f}".format(
                wrist_ros_timestamp
            ),
            "{:.9f}".format(
                camera_time_error
            ),
            "{:.9f}".format(
                state_ros_timestamp
            ),
            "{:.9f}".format(
                action_ros_timestamp
            ),
            "{:.9f}".format(ee_ros_timestamp),
            "{:.9f}".format(
                state_time_error
            ),
            "{:.9f}".format(
                action_time_error
            ),
            "{:.9f}".format(ee_time_error),
            "front/{}".format(front_name),
            "wrist/{}".format(wrist_name),
        ]

        row += follower_copy.tolist()
        row += action_copy.tolist()
        row += ee_copy.tolist()

        self.csv_writer.writerow(row)
        self.csv_file.flush()

        self.num_frames += 1

    def save(self) -> Path:
        if self.closed:
            raise RuntimeError(
                "Episode writer is already closed"
            )

        if self.num_frames <= 0:
            raise RuntimeError(
                "Cannot save zero-frame episode"
            )

        self.csv_file.close()

        np.savez_compressed(
            str(self.temp_dir / "data.npz"),
            timestamp=np.asarray(
                self.timestamp,
                dtype=np.float64,
            ),
            unix_timestamp=np.asarray(
                self.unix_timestamp,
                dtype=np.float64,
            ),
            image_ros_timestamp=np.asarray(
                self.image_ros_timestamp,
                dtype=np.float64,
            ),
            front_ros_timestamp=np.asarray(
                self.front_ros_timestamp,
                dtype=np.float64,
            ),
            wrist_ros_timestamp=np.asarray(
                self.wrist_ros_timestamp,
                dtype=np.float64,
            ),
            camera_time_error=np.asarray(
                self.camera_time_error,
                dtype=np.float32,
            ),
            state_ros_timestamp=np.asarray(
                self.state_ros_timestamp,
                dtype=np.float64,
            ),
            action_ros_timestamp=np.asarray(
                self.action_ros_timestamp,
                dtype=np.float64,
            ),
            ee_ros_timestamp=np.asarray(self.ee_ros_timestamp, dtype=np.float64),
            state_time_error=np.asarray(
                self.state_time_error,
                dtype=np.float32,
            ),
            action_time_error=np.asarray(
                self.action_time_error,
                dtype=np.float32,
            ),
            ee_time_error=np.asarray(self.ee_time_error, dtype=np.float32),
            observation_state=np.asarray(
                self.observation_state,
                dtype=np.float32,
            ),
            action=np.asarray(
                self.action,
                dtype=np.float32,
            ),
            observation_ee=np.asarray(self.observation_ee, dtype=np.float32),
        )

        duration = (
            float(self.timestamp[-1])
            if self.timestamp
            else 0.0
        )
        capture_span = (
            float(
                self.image_ros_timestamp[-1]
                - self.image_ros_timestamp[0]
            )
            if self.num_frames >= 2
            else 0.0
        )
        effective_fps = (
            float(self.num_frames - 1) / capture_span
            if capture_span > 0.0
            else float(self.args.fps)
        )
        frame_intervals = np.diff(
            np.asarray(
                self.image_ros_timestamp,
                dtype=np.float64,
            )
        )
        positive_intervals = frame_intervals[
            frame_intervals > 0.0
        ]
        median_frame_interval = (
            float(np.median(positive_intervals))
            if positive_intervals.size
            else 0.0
        )
        self.effective_fps = effective_fps

        write_json(
            self.temp_dir / "meta.json",
            {
                "format": (
                    "piper_raw_fixed_rate_synced_v6_ee_pose"
                ),
                "episode_name": (
                    self.episode_name
                ),
                "task": self.args.task,
                "fps": effective_fps,
                "configured_fps": self.args.fps,
                "effective_fps": effective_fps,
                "sampling_strategy": "fixed_rate_nearest_timestamp",
                "camera_frame_reuse_enabled": True,
                "median_frame_interval_seconds": (
                    median_frame_interval
                ),
                "num_frames": self.num_frames,
                "duration_seconds": duration,
                "state_dim": (
                    self.args.state_dim
                ),
                "value_names": (
                    self.value_names
                ),
                "observation_state_source": (
                    self.args.follower_topic
                ),
                "observation_ee_source": self.args.ee_topic,
                "ee_position_unit": "m",
                "ee_observation_dim": 8,
                "observation_ee_layout": [
                    "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw", "gripper_m",
                ],
                "observation_ee_orientation": "quaternion_xyzw",
                "observation_ee_gripper_source": self.args.follower_topic,
                "action_source": (
                    self.args.action_topic
                ),
                "raw_leader_topic": (
                    self.args.leader_topic
                ),
                "sync_tolerance_seconds": (
                    self.args.sync_tolerance
                ),
                "sync_wait_seconds": (
                    self.args.sync_wait
                ),
                "camera_sync_tolerance_seconds": (
                    self.args.camera_sync_tolerance
                ),
                "camera_sync_wait_seconds": (
                    self.args.camera_sync_wait
                ),
                "camera_target_tolerance_seconds": (
                    self.args.camera_target_tolerance
                ),
                "camera_fourcc_requested": (
                    self.args.camera_fourcc
                ),
                "camera_fps_requested": (
                    self.args.camera_fps
                ),
                "front_capture_fps": getattr(
                    self,
                    "front_capture_fps",
                    0.0,
                ),
                "wrist_capture_fps": getattr(
                    self,
                    "wrist_capture_fps",
                    0.0,
                ),
                "dropped_sync_frames": getattr(
                    self,
                    "dropped_sync_frames",
                    0,
                ),
                "dropped_camera_sync_frames": getattr(
                    self,
                    "dropped_camera_sync_frames",
                    0,
                ),
                "dropped_schedule_frames": getattr(
                    self,
                    "dropped_schedule_frames",
                    0,
                ),
                "reused_front_frames": getattr(
                    self,
                    "reused_front_frames",
                    0,
                ),
                "reused_wrist_frames": getattr(
                    self,
                    "reused_wrist_frames",
                    0,
                ),
                "camera_time_error_mean_seconds": (
                    float(np.mean(self.camera_time_error))
                ),
                "camera_time_error_max_seconds": (
                    float(np.max(self.camera_time_error))
                ),
                "front_camera": (
                    self.args.front_camera
                ),
                "wrist_camera": (
                    self.args.wrist_camera
                ),
                "image_width": (
                    self.args.width
                ),
                "image_height": (
                    self.args.height
                ),
                "image_encoding": "jpg",
                "repo_id": self.args.repo_id,
                "robot_type": (
                    self.args.robot_type
                ),
                "started_unix_timestamp": (
                    self.start_unix
                ),
                "saved_unix_timestamp": (
                    time.time()
                ),
                "successful": True,
            },
        )

        partial_csv = (
            self.temp_dir
            / "frames.partial.csv"
        )
        final_csv = (
            self.temp_dir
            / "frames.csv"
        )
        partial_csv.rename(final_csv)

        status_path = (
            self.temp_dir
            / "recording_status.json"
        )
        if status_path.exists():
            status_path.unlink()

        self.temp_dir.rename(
            self.final_dir
        )
        self.closed = True

        return self.final_dir

    def discard(self) -> None:
        if self.closed:
            return

        try:
            self.csv_file.close()
        except Exception:
            pass

        shutil.rmtree(
            str(self.temp_dir),
            ignore_errors=True,
        )
        self.closed = True

    def leave_for_recovery(
        self,
        reason: str,
    ) -> None:
        if self.closed:
            return

        try:
            self.csv_file.flush()
            self.csv_file.close()
        except Exception:
            pass

        try:
            write_json(
                self.temp_dir
                / "recording_status.json",
                {
                    "status": "incomplete",
                    "episode_name": (
                        self.episode_name
                    ),
                    "task": self.args.task,
                    "num_frames": (
                        self.num_frames
                    ),
                    "reason": reason,
                    "updated_unix_timestamp": (
                        time.time()
                    ),
                },
            )
        except Exception:
            pass

        self.closed = True
