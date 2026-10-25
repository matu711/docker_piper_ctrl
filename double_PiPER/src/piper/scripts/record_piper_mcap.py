#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PiPER leader-follower recorder with timestamp synchronization and MCAP.

Dataset semantics:
    observation_state = follower measured joints (6) + gripper
    action            = command actually sent to follower (6) + gripper

Synchronization:
    - Each ROS state/action message keeps its header.stamp.
    - The two cameras are captured continuously on independent threads.
    - Camera frames are paired by the nearest capture timestamps.
    - One fixed-grid image reference time is computed for each 20 Hz sample.
    - The state/action messages nearest to that image time are selected.
    - A frame is NOT saved when either time error exceeds --sync-tolerance.

MCAP:
    - raw leader topic (for diagnostics)
    - applied follower command
    - follower measured state
    - front compressed image
    - wrist compressed image

Controls:
    r : start episode
    s : save episode, then return follower to initial pose
    v : save episode without moving the follower
    x : discard episode
    q : quit; discard active unfinished episode
"""

import argparse
import copy
import csv
import json
import numbers
import shutil
import sys
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
from std_srvs.srv import Trigger

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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Record synchronized PiPER leader-follower demonstrations."
        )
    )

    # Raw leader is kept for MCAP/debugging only.
    parser.add_argument(
        "--leader-topic",
        default="/leader/joint_grip_pose",
    )

    # Dataset action: command actually sent to the follower.
    parser.add_argument(
        "--action-topic",
        default="/follower/applied_joint_cmd",
    )

    # Dataset observation.state: measured follower state.
    parser.add_argument(
        "--follower-topic",
        default="/follower/current_joint_obs",
    )
    parser.add_argument(
        "--ee-topic",
        default="/follower/end_pose",
        help="Measured follower geometry_msgs/PoseStamped topic.",
    )
    parser.add_argument(
        "--ee-euler-topic",
        default="/follower/end_pose_euler",
        help="Measured follower PiperEulerPose topic mirrored to MCAP.",
    )

    parser.add_argument("--front-camera", default="/dev/video4")
    parser.add_argument("--wrist-camera", default="/dev/video6")

    parser.add_argument(
        "--camera-backend",
        choices=["v4l2", "default"],
        default="v4l2",
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument(
        "--fps",
        type=int,
        default=20,
        help="Fixed dataset sampling rate in Hz.",
    )
    parser.add_argument(
        "--camera-fps",
        type=int,
        default=30,
        help=(
            "Capture rate requested from each camera before selecting "
            "fixed-rate dataset frames."
        ),
    )
    parser.add_argument("--jpeg-quality", type=int, default=95)
    parser.add_argument(
        "--camera-fourcc",
        default="MJPG",
        help=(
            "FourCC requested from both cameras. MJPG reduces USB bandwidth "
            "and is recommended for two 640x480 cameras."
        ),
    )

    parser.add_argument("--state-dim", type=int, default=7)
    parser.add_argument("--vector-start", type=int, default=0)

    # This replaces the old state-max-age mechanism.
    parser.add_argument(
        "--sync-tolerance",
        type=float,
        default=0.03,
        help=(
            "Maximum allowed absolute timestamp error in seconds "
            "between image time and state/action time."
        ),
    )
    parser.add_argument(
        "--sync-wait",
        type=float,
        default=0.025,
        help=(
            "Maximum time to wait for a ROS sample newer than the "
            "camera timestamp before selecting the nearest sample."
        ),
    )
    parser.add_argument(
        "--camera-sync-tolerance",
        type=float,
        default=0.03,
        help=(
            "Maximum allowed timestamp difference in seconds "
            "between paired front and wrist frames."
        ),
    )
    parser.add_argument(
        "--camera-sync-wait",
        type=float,
        default=0.10,
        help=(
            "Maximum time to wait for a closer front/wrist frame pair."
        ),
    )
    parser.add_argument(
        "--camera-target-tolerance",
        type=float,
        default=0.10,
        help=(
            "Maximum allowed absolute timestamp error in seconds "
            "between either camera and the fixed-rate sample time."
        ),
    )
    parser.add_argument(
        "--camera-buffer-size",
        type=int,
        default=8,
        help="Number of timestamped frames kept per camera.",
    )
    parser.add_argument(
        "--ros-buffer-size",
        type=int,
        default=500,
        help="Number of timestamped ROS messages kept per stream.",
    )

    parser.add_argument(
        "--root",
        type=Path,
        default=Path("./piper_raw_dataset"),
    )
    parser.add_argument(
        "--repo-id",
        default="local/piper_leader_follower",
    )
    parser.add_argument(
        "--robot-type",
        default="piper_single_arm",
    )
    parser.add_argument(
        "--task",
        default="perform the demonstrated manipulation task",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
    )

    # Compatibility with the existing launch file.
    parser.add_argument(
        "--no-video",
        action="store_true",
    )

    return parser.parse_args(
        rospy.myargv(argv=sys.argv)[1:]
    )


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


class RosMcapMirror:
    """
    Mirror a ROS topic into MCAP for diagnostics.
    It is not used to construct the learning frame.
    """

    def __init__(
        self,
        topic: str,
        label: str,
        mcap_recorder: McapEpisodeRecorder,
    ):
        self.topic = topic
        self.label = label
        self.mcap_recorder = mcap_recorder

        msg_class, real_topic, _ = rostopic.get_topic_class(
            topic,
            blocking=True,
        )

        if msg_class is None or real_topic is None:
            raise RuntimeError(
                "ROS topic not found: {}".format(topic)
            )

        self.real_topic = real_topic

        self.subscriber = rospy.Subscriber(
            real_topic,
            msg_class,
            self.callback,
            queue_size=10,
        )

    def callback(self, msg: Any) -> None:
        self.mcap_recorder.write_ros_message(
            self.real_topic,
            copy.deepcopy(msg),
        )


def open_camera(
    device: str,
    backend_name: str,
    width: int,
    height: int,
    fps: int,
    fourcc: str,
) -> cv2.VideoCapture:
    backend = (
        cv2.CAP_V4L2
        if backend_name == "v4l2"
        else cv2.CAP_ANY
    )

    cap = cv2.VideoCapture(device, backend)

    if not cap.isOpened():
        raise RuntimeError(
            "Camera could not be opened: {}".format(
                device
            )
        )

    requested_fourcc = cv2.VideoWriter_fourcc(*fourcc)
    cap.set(cv2.CAP_PROP_FOURCC, requested_fourcc)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    valid_frame = None

    for _ in range(15):
        ok, frame = cap.read()
        if ok and frame is not None:
            valid_frame = frame
        time.sleep(0.01)

    if valid_frame is None:
        cap.release()
        raise RuntimeError(
            "No frames received from camera: {}".format(
                device
            )
        )

    actual_fourcc_value = int(
        cap.get(cv2.CAP_PROP_FOURCC)
    )
    actual_fourcc = "".join(
        chr(
            (actual_fourcc_value >> (8 * index))
            & 0xFF
        )
        for index in range(4)
    )
    print(
        "Camera {}: {}x{} @ reported {:.2f} FPS, FourCC={}".format(
            device,
            int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            cap.get(cv2.CAP_PROP_FPS),
            actual_fourcc,
        )
    )

    return cap


class ThreadedCamera:
    """Continuously capture one camera without blocking the main loop."""

    def __init__(
        self,
        cap: cv2.VideoCapture,
        width: int,
        height: int,
        label: str,
        buffer_size: int,
    ):
        self.cap = cap
        self.width = width
        self.height = height
        self.label = label
        self.condition = threading.Condition()
        self.frames = deque(maxlen=buffer_size)
        self.capture_timestamps = deque(maxlen=120)
        self.running = True
        self.error = None
        self.sequence = 0
        self.thread = threading.Thread(
            target=self._capture_loop,
            name="{}_camera_capture".format(label),
            daemon=True,
        )
        self.thread.start()

    def _capture_loop(self) -> None:
        consecutive_failures = 0

        while self.running and not rospy.is_shutdown():
            before_ros = rospy.Time.now().to_sec()
            before_monotonic = time.monotonic()
            ok, frame = self.cap.read()
            after_monotonic = time.monotonic()
            after_ros = rospy.Time.now().to_sec()

            if not ok or frame is None:
                consecutive_failures += 1
                if consecutive_failures < 5:
                    time.sleep(0.005)
                    continue

                with self.condition:
                    self.error = RuntimeError(
                        "Failed to read {} camera frame 5 times".format(
                            self.label
                        )
                    )
                    self.condition.notify_all()
                return

            consecutive_failures = 0

            if (
                frame.shape[1] != self.width
                or frame.shape[0] != self.height
            ):
                frame = cv2.resize(
                    frame,
                    (self.width, self.height),
                    interpolation=cv2.INTER_AREA,
                )

            # OpenCV does not expose the V4L2 hardware timestamp here.
            # The midpoint of the blocking read is the least biased
            # software-side estimate available to this recorder.
            ros_timestamp = 0.5 * (before_ros + after_ros)
            monotonic_timestamp = 0.5 * (
                before_monotonic + after_monotonic
            )

            with self.condition:
                item = (
                    self.sequence,
                    frame,
                    ros_timestamp,
                    monotonic_timestamp,
                )
                self.sequence += 1
                self.frames.append(item)
                self.capture_timestamps.append(
                    monotonic_timestamp
                )
                self.condition.notify_all()

    def wait_ready(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        with self.condition:
            while not self.frames and self.error is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self.condition.wait(timeout=remaining)

            if self.error is not None:
                raise self.error
            if not self.frames:
                raise RuntimeError(
                    "Timed out waiting for {} camera".format(self.label)
                )

    def frames_after(self, sequence: int) -> List[Tuple]:
        with self.condition:
            if self.error is not None:
                raise self.error
            return [item for item in self.frames if item[0] > sequence]

    def frame_snapshot(self) -> List[Tuple]:
        with self.condition:
            if self.error is not None:
                raise self.error
            return list(self.frames)

    def measured_fps(self) -> float:
        with self.condition:
            if len(self.capture_timestamps) < 2:
                return 0.0
            span = (
                self.capture_timestamps[-1]
                - self.capture_timestamps[0]
            )
            if span <= 0.0:
                return 0.0
            return (
                float(len(self.capture_timestamps) - 1)
                / span
            )

    def stop(self) -> None:
        self.running = False
        self.thread.join(timeout=0.5)
        self.cap.release()
        self.thread.join(timeout=1.0)


def synchronized_camera_pair(
    front_camera: ThreadedCamera,
    wrist_camera: ThreadedCamera,
    last_front_sequence: int,
    last_wrist_sequence: int,
    target_ros_timestamp: float,
    tolerance: float,
    target_tolerance: float,
    wait_seconds: float,
    allow_reuse: bool = True,
) -> Tuple[Optional[Tuple], Optional[Tuple], float, bool]:
    """Return the unused camera pair nearest a fixed sample time."""
    deadline = time.monotonic() + wait_seconds
    best_pair = None
    best_error = float("inf")
    best_target_error = float("inf")
    best_score = (1, float("inf"), float("inf"))

    while not rospy.is_shutdown():
        if allow_reuse:
            front_frames = front_camera.frame_snapshot()
            wrist_frames = wrist_camera.frame_snapshot()
        else:
            front_frames = front_camera.frames_after(last_front_sequence)
            wrist_frames = wrist_camera.frames_after(last_wrist_sequence)

        for front_frame in front_frames:
            for wrist_frame in wrist_frames:
                error = abs(front_frame[2] - wrist_frame[2])
                target_error = max(
                    abs(front_frame[2] - target_ros_timestamp),
                    abs(wrist_frame[2] - target_ros_timestamp),
                )
                candidate_is_valid = (
                    error <= tolerance
                    and target_error <= target_tolerance
                )
                score = (
                    0 if candidate_is_valid else 1,
                    target_error,
                    error,
                )
                if score < best_score:
                    best_pair = (front_frame, wrist_frame)
                    best_error = error
                    best_target_error = target_error
                    best_score = score

        streams_reached_target = (
            bool(front_frames)
            and bool(wrist_frames)
            and front_frames[-1][2] >= target_ros_timestamp
            and wrist_frames[-1][2] >= target_ros_timestamp
        )
        pair_is_valid = (
            best_pair is not None
            and best_error <= tolerance
            and best_target_error <= target_tolerance
        )

        if pair_is_valid and (
            allow_reuse or streams_reached_target
        ):
            return best_pair[0], best_pair[1], best_error, True

        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            if best_pair is None:
                # A camera frame period can be longer than the requested
                # loop period. This is a normal temporary condition, not a
                # fatal camera failure. The capture threads report repeated
                # read failures separately through frames_after().
                return None, None, float("inf"), False
            return (
                best_pair[0],
                best_pair[1],
                best_error,
                pair_is_valid,
            )

        time.sleep(min(0.002, remaining))

    raise RuntimeError("ROS shutdown while waiting for camera frames")


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


def create_dataset_metadata(
    root: Path,
    args: argparse.Namespace,
    value_names: List[str],
) -> None:
    metadata = {
        "format": (
            "piper_raw_fixed_rate_synced_v6_ee_pose"
        ),
        "description": (
            "PiPER leader-follower demonstrations "
            "synchronized by ROS timestamps."
        ),
        "task": args.task,
        "fps": args.fps,
        "configured_fps": args.fps,
        "sampling_strategy": "fixed_rate_nearest_timestamp",
        "camera_frame_reuse_enabled": True,
        "state_dim": args.state_dim,
        "value_names": value_names,
        "observation_state_source": (
            args.follower_topic
        ),
        "observation_ee_source": args.ee_topic,
        "ee_position_unit": "m",
        "ee_observation_dim": 8,
        "observation_ee_layout": [
            "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw", "gripper_m",
        ],
        "observation_ee_orientation": "quaternion_xyzw",
        "observation_ee_gripper_source": args.follower_topic,
        "action_source": (
            args.action_topic
        ),
        "raw_leader_topic": (
            args.leader_topic
        ),
        "sync_tolerance_seconds": (
            args.sync_tolerance
        ),
        "sync_wait_seconds": (
            args.sync_wait
        ),
        "camera_sync_tolerance_seconds": (
            args.camera_sync_tolerance
        ),
        "camera_sync_wait_seconds": (
            args.camera_sync_wait
        ),
        "camera_target_tolerance_seconds": (
            args.camera_target_tolerance
        ),
        "camera_fourcc_requested": (
            args.camera_fourcc
        ),
        "camera_fps_requested": (
            args.camera_fps
        ),
        "front_camera": (
            args.front_camera
        ),
        "wrist_camera": (
            args.wrist_camera
        ),
        "image_width": args.width,
        "image_height": args.height,
        "repo_id": args.repo_id,
        "robot_type": args.robot_type,
        "created_unix_timestamp": (
            time.time()
        ),
    }

    path = root / "dataset_meta.json"

    if path.exists():
        write_json(
            root
            / "last_recording_config.json",
            metadata,
        )
    else:
        write_json(
            path,
            metadata,
        )


def make_preview(
    front_bgr: np.ndarray,
    wrist_bgr: np.ndarray,
    recording: bool,
    episode_name: str,
    episode_frames: int,
    saved_episodes: int,
    follower: Optional[np.ndarray],
    action: Optional[np.ndarray],
    ee: Optional[np.ndarray],
    sync_ok: bool,
    camera_error: float,
    state_error: float,
    action_error: float,
    ee_error: float,
) -> np.ndarray:
    preview = np.concatenate(
        [front_bgr, wrist_bgr],
        axis=1,
    )

    status = (
        "RECORDING"
        if recording
        else "READY"
    )
    sync_text = (
        "SYNC OK"
        if sync_ok
        else "SYNC WAIT/DROP"
    )

    lines = [
        "{} | {} | episode={} | frames={} | saved={}".format(
            status,
            sync_text,
            episode_name,
            episode_frames,
            saved_episodes,
        ),
        (
            "sync error: camera={:.1f} ms state={:.1f} ms "
            "action={:.1f} ms EE={:.1f} ms"
        ).format(
            camera_error * 1000.0
            if np.isfinite(camera_error)
            else float("inf"),
            state_error * 1000.0
            if np.isfinite(state_error)
            else float("inf"),
            action_error * 1000.0
            if np.isfinite(action_error)
            else float("inf"),
            ee_error * 1000.0
            if np.isfinite(ee_error)
            else float("inf"),
        ),
        "p: register   h: initial pose   r: teach+record   c: resume",
        "r: record   s: save+home   v: save only   x: discard   q: quit",
    ]

    if follower is not None:
        lines.append(
            "follower/state: {}".format(
                np.array2string(
                    follower,
                    precision=3,
                    suppress_small=True,
                )
            )
        )

    if action is not None:
        lines.append(
            "applied/action: {}".format(
                np.array2string(
                    action,
                    precision=3,
                    suppress_small=True,
                )
            )
        )

    if ee is not None:
        lines.append(
            "EE [m,quat,grip m]: {}".format(
                np.array2string(ee, precision=3, suppress_small=True)
            )
        )

    y = 24

    for text in lines:
        cv2.putText(
            preview,
            text,
            (8, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            preview,
            text,
            (8, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 0),
            1,
            cv2.LINE_AA,
        )
        y += 25

    return preview


def count_saved_episodes(
    root: Path,
) -> int:
    return len(
        [
            path
            for path in root.iterdir()
            if (
                path.is_dir()
                and not path.name.startswith(
                    ".tmp_"
                )
            )
        ]
    )


def main() -> int:
    args = parse_args()

    if args.state_dim <= 0:
        raise ValueError(
            "--state-dim must be positive"
        )
    if args.state_dim < 7:
        raise ValueError("--state-dim must be at least 7 (six joints + gripper)")

    if args.vector_start < 0:
        raise ValueError(
            "--vector-start must be non-negative"
        )

    if args.fps <= 0:
        raise ValueError(
            "--fps must be positive"
        )

    if args.camera_fps <= 0:
        raise ValueError(
            "--camera-fps must be positive"
        )

    if not (
        0.0 < args.sync_tolerance < 1.0
    ):
        raise ValueError(
            "--sync-tolerance must be "
            "between 0 and 1 second"
        )

    if not (
        0.0 <= args.sync_wait < 1.0
    ):
        raise ValueError(
            "--sync-wait must be "
            "between 0 and 1 second"
        )

    if not (
        0.0 < args.camera_sync_tolerance < 1.0
    ):
        raise ValueError(
            "--camera-sync-tolerance must be between 0 and 1 second"
        )

    if not (
        0.0 <= args.camera_sync_wait < 1.0
    ):
        raise ValueError(
            "--camera-sync-wait must be between 0 and 1 second"
        )

    if not (
        0.0 < args.camera_target_tolerance < 1.0
    ):
        raise ValueError(
            "--camera-target-tolerance must be between 0 and 1 second"
        )

    if args.camera_buffer_size <= 1:
        raise ValueError(
            "--camera-buffer-size must be > 1"
        )

    if len(args.camera_fourcc) != 4:
        raise ValueError(
            "--camera-fourcc must contain exactly 4 characters"
        )

    if args.ros_buffer_size <= 1:
        raise ValueError(
            "--ros-buffer-size must be > 1"
        )

    if not (
        1 <= args.jpeg_quality <= 100
    ):
        raise ValueError(
            "--jpeg-quality must be 1..100"
        )

    args.root = (
        args.root.expanduser().resolve()
    )

    if (
        args.root.exists()
        and args.overwrite
    ):
        shutil.rmtree(
            str(args.root)
        )

    args.root.mkdir(
        parents=True,
        exist_ok=True,
    )

    value_names = make_names(
        args.state_dim
    )

    create_dataset_metadata(
        args.root,
        args,
        value_names,
    )

    rospy.init_node(
        "piper_mcap_recorder",
        anonymous=False,
    )

    if McapRos1Writer is None:
        raise RuntimeError(
            "MCAP support is not installed.\n"
            "Run: python3 -m pip install "
            "mcap mcap-ros1-support"
        )

    mcap_recorder = (
        McapEpisodeRecorder()
    )

    # Cameras are ROS topics continuously.
    front_image_pub = rospy.Publisher(
        "/camera/front/compressed",
        CompressedImage,
        queue_size=1,
    )
    wrist_image_pub = rospy.Publisher(
        "/camera/wrist/compressed",
        CompressedImage,
        queue_size=1,
    )

    # Raw leader is useful for Foxglove diagnostics,
    # but is NOT the dataset action.
    leader_mirror = RosMcapMirror(
        args.leader_topic,
        "raw leader",
        mcap_recorder,
    )

    # Dataset action = actual command sent to follower.
    action_source = RosVectorBuffer(
        args.action_topic,
        "applied action",
        args.ros_buffer_size,
        mcap_recorder,
    )

    # Dataset state = measured follower state.
    follower_source = RosVectorBuffer(
        args.follower_topic,
        "follower state",
        args.ros_buffer_size,
        mcap_recorder,
    )
    ee_source = RosVectorBuffer(
        args.ee_topic,
        "follower end-effector pose",
        args.ros_buffer_size,
        mcap_recorder,
        extractor=extract_ee_pose,
    )
    ee_euler_mirror = RosMcapMirror(
        args.ee_euler_topic,
        "follower end-effector Euler debug",
        mcap_recorder,
    )

    front_cap = open_camera(
        args.front_camera,
        args.camera_backend,
        args.width,
        args.height,
        args.camera_fps,
        args.camera_fourcc,
    )
    try:
        wrist_cap = open_camera(
            args.wrist_camera,
            args.camera_backend,
            args.width,
            args.height,
            args.camera_fps,
            args.camera_fourcc,
        )
    except Exception:
        front_cap.release()
        raise

    front_camera = ThreadedCamera(
        front_cap,
        args.width,
        args.height,
        "front",
        args.camera_buffer_size,
    )
    wrist_camera = None
    try:
        wrist_camera = ThreadedCamera(
            wrist_cap,
            args.width,
            args.height,
            "wrist",
            args.camera_buffer_size,
        )
        front_camera.wait_ready(timeout=2.0)
        wrist_camera.wait_ready(timeout=2.0)
    except Exception:
        front_camera.stop()
        if wrist_camera is not None:
            wrist_camera.stop()
        else:
            wrist_cap.release()
        raise

    cv2.namedWindow(
        "PiPER synchronized recorder",
        cv2.WINDOW_NORMAL,
    )

    next_episode_name = (
        make_episode_name(args.root)
    )
    writer = None
    dropped_sync_frames = 0
    dropped_camera_sync_frames = 0
    dropped_schedule_frames = 0
    reused_front_frames = 0
    reused_wrist_frames = 0
    last_front_sequence = -1
    last_wrist_sequence = -1

    period = 1.0 / float(args.fps)
    next_tick = time.monotonic()

    print("")
    print(
        "PiPER synchronized recorder"
    )
    print(
        "  state  : {}".format(
            args.follower_topic
        )
    )
    print(
        "  action : {}".format(
            args.action_topic
        )
    )
    print("  EE pose: {}".format(args.ee_topic))
    print(
        "  leader : {} (MCAP debug only)".format(
            args.leader_topic
        )
    )
    print(
        "  sync tolerance : {:.1f} ms".format(
            args.sync_tolerance * 1000.0
        )
    )
    print(
        "  camera tolerance: {:.1f} ms".format(
            args.camera_sync_tolerance * 1000.0
        )
    )
    print("")
    print("r: start")
    print("s: save, then return follower to initial pose")
    print("v: save only; do not move follower")
    print("x: discard")
    print("p: save current follower pose as initial pose")
    print("h: return follower to initial pose; leader remains in TEACHING_MODE")
    print("r: verify leader TEACHING_MODE, resume follower, then record")
    print("c: resume leader control without recording")
    print("q: quit")
    print("")

    def call_trigger_async(service_name, label):
        def worker():
            try:
                rospy.wait_for_service(service_name, timeout=1.0)
                response = rospy.ServiceProxy(service_name, Trigger)()
                level = rospy.loginfo if response.success else rospy.logerr
                level("%s: %s", label, response.message)
            except Exception as exc:
                rospy.logerr("%s service failed: %s", label, exc)
        threading.Thread(target=worker, daemon=True).start()

    recording_start = {"pending": False, "ready": False, "error": None}
    recording_start_lock = threading.Lock()

    def prepare_recording_async():
        def worker():
            try:
                rospy.wait_for_service("/leader/enter_teaching_mode", timeout=2.0)
                leader_result = rospy.ServiceProxy("/leader/enter_teaching_mode", Trigger)()
                if not leader_result.success:
                    raise RuntimeError(leader_result.message)
                rospy.wait_for_service("/follower/resume_leader_control", timeout=2.0)
                follower_result = rospy.ServiceProxy("/follower/resume_leader_control", Trigger)()
                if not follower_result.success:
                    raise RuntimeError(follower_result.message)
                with recording_start_lock:
                    recording_start["ready"] = True
                    recording_start["pending"] = False
                rospy.loginfo("Leader is in TEACHING_MODE; follower control resumed. Starting recording.")
            except Exception as exc:
                with recording_start_lock:
                    recording_start["error"] = str(exc)
                    recording_start["pending"] = False
                rospy.logerr("Recording start aborted: %s", exc)
        threading.Thread(target=worker, daemon=True).start()

    try:
        while not rospy.is_shutdown():

            with recording_start_lock:
                should_start_recording = recording_start["ready"] and writer is None
                if should_start_recording:
                    recording_start["ready"] = False
            if should_start_recording:
                next_episode_name = make_episode_name(args.root)
                writer = RawEpisodeWriter(args.root, next_episode_name, args, value_names)
                mcap_recorder.start(writer.temp_dir)
                dropped_sync_frames = 0
                dropped_camera_sync_frames = 0
                dropped_schedule_frames = 0
                reused_front_frames = 0
                reused_wrist_frames = 0
                print("Recording {}...".format(next_episode_name))

            # Fixed-rate learning timestamp. If processing falls behind by
            # more than one period, skip stale ticks instead of bursting old
            # frames into the dataset.
            now_monotonic = time.monotonic()
            if now_monotonic < next_tick:
                time.sleep(next_tick - now_monotonic)
            else:
                lag = now_monotonic - next_tick
                missed_ticks = int(lag // period)
                if missed_ticks > 0:
                    next_tick += missed_ticks * period
                    if writer is not None:
                        dropped_schedule_frames += missed_ticks

            sample_monotonic = next_tick
            ros_clock_offset = (
                rospy.Time.now().to_sec()
                - time.monotonic()
            )
            sample_ros_timestamp = (
                sample_monotonic + ros_clock_offset
            )

            # -------------------------
            # Capture cameras + timestamps
            # -------------------------
            previous_front_sequence = last_front_sequence
            previous_wrist_sequence = last_wrist_sequence
            (
                front_frame,
                wrist_frame,
                camera_error,
                camera_sync_ok,
            ) = synchronized_camera_pair(
                front_camera,
                wrist_camera,
                last_front_sequence,
                last_wrist_sequence,
                sample_ros_timestamp,
                args.camera_sync_tolerance,
                args.camera_target_tolerance,
                args.camera_sync_wait,
            )

            if front_frame is None or wrist_frame is None:
                rospy.logwarn_throttle(
                    2.0,
                    "Waiting for the next front/wrist camera frames",
                )
                time.sleep(0.005)
                next_tick += period
                continue

            (
                last_front_sequence,
                front_bgr,
                front_time,
                front_monotonic,
            ) = front_frame
            (
                last_wrist_sequence,
                wrist_bgr,
                wrist_time,
                wrist_monotonic,
            ) = wrist_frame

            front_frame_reused = (
                last_front_sequence
                == previous_front_sequence
            )
            wrist_frame_reused = (
                last_wrist_sequence
                == previous_wrist_sequence
            )

            front_stamp = rospy.Time.from_sec(front_time)
            wrist_stamp = rospy.Time.from_sec(wrist_time)

            # State/action and the dataset row use the exact 10 Hz grid.
            # Actual camera timestamps remain available in separate fields.
            image_time = sample_ros_timestamp
            image_monotonic = sample_monotonic

            rospy.loginfo_throttle(
                5.0,
                (
                    "Camera capture FPS: front=%.2f wrist=%.2f; "
                    "paired skew=%.1f ms"
                ),
                front_camera.measured_fps(),
                wrist_camera.measured_fps(),
                camera_error * 1000.0,
            )

            front_msg, front_jpeg = (
                make_compressed_image(
                    front_bgr,
                    front_stamp,
                    "front_camera",
                    args.jpeg_quality,
                )
            )
            wrist_msg, wrist_jpeg = (
                make_compressed_image(
                    wrist_bgr,
                    wrist_stamp,
                    "wrist_camera",
                    args.jpeg_quality,
                )
            )

            # Camera ROS topics are visible continuously.
            front_image_pub.publish(front_msg)
            wrist_image_pub.publish(wrist_msg)

            # -------------------------
            # Timestamp synchronization
            # -------------------------
            deadline = (
                time.monotonic()
                + args.sync_wait
            )

            action_source.wait_until_at_least(
                image_time,
                deadline,
            )
            follower_source.wait_until_at_least(
                image_time,
                deadline,
            )
            ee_source.wait_until_at_least(image_time, deadline)

            (
                raw_action,
                action_stamp,
                action_error,
            ) = action_source.nearest(
                image_time
            )

            (
                raw_follower,
                state_stamp,
                state_error,
            ) = follower_source.nearest(
                image_time
            )
            raw_ee, ee_stamp, ee_error = ee_source.nearest(image_time)

            action = None
            follower = None
            ee = None

            if raw_action is not None:
                action = select_vector(
                    raw_action,
                    args.vector_start,
                    args.state_dim,
                    "action",
                )

            if raw_follower is not None:
                follower = select_vector(
                    raw_follower,
                    args.vector_start,
                    args.state_dim,
                    "follower",
                )
            if raw_ee is not None and raw_ee.size == 7 and follower is not None:
                # The driver publishes state and EE with one Timer stamp;
                # gripper is therefore the measured state paired to this EE pose.
                ee = np.concatenate((raw_ee, follower[6:7])).astype(np.float32, copy=False)

            sync_ok = (
                camera_sync_ok
                and action is not None
                and follower is not None
                and ee is not None
                and abs(action_error)
                <= args.sync_tolerance
                and abs(state_error)
                <= args.sync_tolerance
                and abs(ee_error)
                <= args.sync_tolerance
            )

            # -------------------------
            # Save one synchronized frame
            # -------------------------
            if writer is not None:
                if sync_ok:
                    if front_frame_reused:
                        reused_front_frames += 1
                    if wrist_frame_reused:
                        reused_wrist_frames += 1
                    writer.add_frame(
                        front_jpeg=front_jpeg,
                        wrist_jpeg=wrist_jpeg,
                        follower=follower,
                        action=action,
                        ee=ee,
                        image_ros_timestamp=(
                            image_time
                        ),
                        front_ros_timestamp=(
                            front_time
                        ),
                        wrist_ros_timestamp=(
                            wrist_time
                        ),
                        image_monotonic_timestamp=(
                            image_monotonic
                        ),
                        camera_time_error=(
                            camera_error
                        ),
                        state_ros_timestamp=(
                            state_stamp
                        ),
                        action_ros_timestamp=(
                            action_stamp
                        ),
                        ee_ros_timestamp=ee_stamp,
                        state_time_error=(
                            state_error
                        ),
                        action_time_error=(
                            action_error
                        ),
                        ee_time_error=ee_error,
                    )

                    # Exact camera messages saved in MCAP.
                    mcap_recorder.write_ros_message(
                        "/camera/front/compressed",
                        front_msg,
                    )
                    mcap_recorder.write_ros_message(
                        "/camera/wrist/compressed",
                        wrist_msg,
                    )

                else:
                    dropped_sync_frames += 1
                    if not camera_sync_ok:
                        dropped_camera_sync_frames += 1
                    rospy.logwarn_throttle(
                        1.0,
                        (
                            "Frame dropped by sync: camera=%.1f ms "
                            "state=%.1f ms action=%.1f ms EE=%.1f ms "
                            "(camera limit=%.1f ms, ROS limit=%.1f ms)"
                        ),
                        camera_error * 1000.0,
                        abs(state_error) * 1000.0,
                        abs(action_error) * 1000.0,
                        abs(ee_error) * 1000.0,
                        args.camera_sync_tolerance
                        * 1000.0,
                        args.sync_tolerance
                        * 1000.0,
                    )

            active_episode = (
                writer.episode_name
                if writer is not None
                else next_episode_name
            )
            active_frames = (
                writer.num_frames
                if writer is not None
                else 0
            )

            preview = make_preview(
                front_bgr=front_bgr,
                wrist_bgr=wrist_bgr,
                recording=(
                    writer is not None
                ),
                episode_name=(
                    active_episode
                ),
                episode_frames=(
                    active_frames
                ),
                saved_episodes=(
                    count_saved_episodes(
                        args.root
                    )
                ),
                follower=follower,
                action=action,
                ee=ee,
                sync_ok=sync_ok,
                camera_error=camera_error,
                state_error=state_error,
                action_error=action_error,
                ee_error=ee_error,
            )

            cv2.imshow(
                "PiPER synchronized recorder",
                preview,
            )

            key = cv2.waitKey(1) & 0xFF

            if key == ord("r"):
                if writer is not None:
                    print(
                        "Already recording."
                    )

                elif not (
                    action_source.has_data()
                    and follower_source.has_data()
                    and ee_source.has_data()
                ):
                    print(
                        "Cannot start: waiting for "
                        "state/action/EE topics."
                    )
                elif recording_start["pending"]:
                    print("Waiting for Leader TEACHING_MODE transition...")
                else:
                    with recording_start_lock:
                        recording_start["pending"] = True
                        recording_start["error"] = None
                    print("Keep a hand on the Leader. Switching to TEACHING_MODE...")
                    prepare_recording_async()

            elif key in (ord("s"), ord("v")):
                if writer is None:
                    print(
                        "No episode is recording."
                    )

                elif writer.num_frames == 0:
                    print(
                        "No synchronized frames "
                        "have been recorded."
                    )

                else:
                    frame_count = (
                        writer.num_frames
                    )
                    episode_name = (
                        writer.episode_name
                    )

                    writer.front_capture_fps = (
                        front_camera.measured_fps()
                    )
                    writer.wrist_capture_fps = (
                        wrist_camera.measured_fps()
                    )
                    writer.dropped_sync_frames = (
                        dropped_sync_frames
                    )
                    writer.dropped_camera_sync_frames = (
                        dropped_camera_sync_frames
                    )
                    writer.dropped_schedule_frames = (
                        dropped_schedule_frames
                    )
                    writer.reused_front_frames = (
                        reused_front_frames
                    )
                    writer.reused_wrist_frames = (
                        reused_wrist_frames
                    )
                    mcap_recorder.finish()
                    final_dir = writer.save()

                    print(
                        "Saved {}: {} frames".format(
                            episode_name,
                            frame_count,
                        )
                    )
                    print(
                        "Dropped by sync: {}".format(
                            dropped_sync_frames
                        )
                    )
                    print(
                        "  camera-pair drops: {}".format(
                            dropped_camera_sync_frames
                        )
                    )
                    print(
                        "  schedule drops: {}".format(
                            dropped_schedule_frames
                        )
                    )
                    print(
                        "  reused camera frames: front={} wrist={}".format(
                            reused_front_frames,
                            reused_wrist_frames,
                        )
                    )
                    print(
                        "  effective FPS: {:.3f}".format(
                            writer.effective_fps
                        )
                    )
                    print(
                        "  -> {}".format(
                            final_dir
                        )
                    )
                    print(
                        "  -> {}".format(
                            final_dir
                            / (
                                episode_name
                                + ".mcap"
                            )
                        )
                    )

                    writer = None
                    next_episode_name = (
                        make_episode_name(
                            args.root
                        )
                    )
                    if key == ord("s"):
                        call_trigger_async(
                            "/follower/go_initial_pose",
                            "Go initial pose after save",
                        )

            elif key == ord("x"):
                if writer is None:
                    print(
                        "No episode is recording."
                    )
                else:
                    name = (
                        writer.episode_name
                    )
                    mcap_recorder.discard()
                    writer.discard()
                    writer = None
                    print(
                        "Discarded {}".format(
                            name
                        )
                    )

            elif key == ord("p"):
                call_trigger_async("/follower/save_initial_pose", "Save initial pose")

            elif key == ord("h"):
                call_trigger_async("/follower/go_initial_pose", "Go initial pose")

            elif key == ord("c"):
                call_trigger_async("/follower/resume_leader_control", "Resume leader control")

            elif key == ord("q"):
                if writer is not None:
                    name = (
                        writer.episode_name
                    )
                    mcap_recorder.discard()
                    writer.discard()
                    writer = None
                    print(
                        "Discarded unfinished "
                        "{}".format(name)
                    )
                break

            # -------------------------
            # Maintain target FPS
            # -------------------------
            next_tick += period

    except KeyboardInterrupt:
        if writer is not None:
            mcap_recorder.finish_incomplete()
            writer.leave_for_recovery(
                "KeyboardInterrupt or ROS shutdown"
            )
        return 130

    except Exception as exc:
        if writer is not None:
            mcap_recorder.finish_incomplete()
            writer.leave_for_recovery(
                "{}: {}".format(
                    type(exc).__name__,
                    exc,
                )
            )
        raise

    finally:
        front_camera.stop()
        wrist_camera.stop()
        cv2.destroyAllWindows()

    print("Recorder finished.")
    print(
        "Dataset root: {}".format(
            args.root
        )
    )

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
