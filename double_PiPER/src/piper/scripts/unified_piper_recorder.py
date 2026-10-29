#!/usr/bin/env python3
"""Shared recorder derived from collect_dual_piper.launch / record_dual_piper.py.

No robot control clients. Configuration and legacy single storage are adapters.
"""
import codecs
import json
import itertools
import math
import os
import select
import signal
import shutil
import sys
import termios
import threading
import time
import tty
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse

from piper_recording_common import McapEpisodeRecorder, RosVectorBuffer, make_compressed_image
from mcap2_camera import CameraCapture, _open, frame_time
from unified_piper_config import configuration
from unified_piper_gui import draw_details
from unified_piper_storage import STREAMS, NAMES, atomic_json, saved_episode_folders


CAMERAS = ("front", "wrist1", "wrist2")
DATASET_ROOT = Path.home() / "piper_dataset"


def capture_worker(devices, config, stop, connection):
    """Single capture owner; preserve each camera's native frame dimensions."""
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    os.environ["OPENCV_VIDEOIO_V4L_SELECT_TIMEOUT"] = "10"
    cv2.setNumThreads(1)
    cameras, settings, last_frame_times = [], {}, {}
    try:
        # Match the successful triple-camera test: configure all before reads.
        for name, device in devices:
            requested = dict(config, **config["cameras"][name])
            cap, settings[name] = _open(device, requested, stop)
            cameras.append((name, device, cap))
        connection.send(("settings", settings))
        sequence = 0
        while not stop.is_set():
            started, batch = time.monotonic(), {}
            changed = sequence == 0
            for name, device, cap in cameras:
                if stop.is_set():
                    return
                before_ros, before_mono = time.time(), time.monotonic()
                ok, frame = cap.read()
                after_mono, after_ros = time.monotonic(), time.time()
                if stop.is_set():
                    return
                if not ok or frame is None:
                    raise RuntimeError("Could not read a frame from "+device)
                # Keep the age of past kernel timestamps. A short validity
                # window would relabel very old images with a fresh midpoint.
                stamp, mono, source = frame_time(cap, before_ros, before_mono, after_ros, after_mono,
                                                 float("inf"))
                if source.startswith("v4l2_") and mono <= last_frame_times.get(name, -float("inf")):
                    raise RuntimeError("Camera frame timestamp did not advance: "+device)
                last_frame_times[name] = mono
                height, width = frame.shape[:2]
                if settings[name]["width"] != width or settings[name]["height"] != height:
                    changed = True
                settings[name].update(width=width, height=height, output_width=width,
                                      output_height=height, timestamp_source=source)
                batch[name] = (sequence, frame, stamp, mono)
            if changed:
                connection.send(("settings", settings))
            connection.send(("frames", batch))
            sequence += 1
            stop.wait(max(0., 1./config["fps"]-(time.monotonic()-started)))
    except Exception as exc:
        try:
            connection.send(("error", str(exc)))
        except (EOFError, OSError):
            pass
    finally:
        for _, _, cap in cameras:
            cap.release()
        connection.close()


def synchronized_images(snapshots, target, now, mono_now, last_sequence,
                        target_tolerance, camera_tolerance, max_age, cameras=CAMERAS):
    candidates = []
    for name in cameras:
        valid = [frame for frame in snapshots[name]
                 if frame[1] is not None
                 and math.isfinite(frame[2]) and math.isfinite(frame[3])
                 and abs(frame[2]-target) <= target_tolerance+1e-9
                 and -1e-6 <= now-frame[2] <= max_age
                 and 0 <= mono_now-frame[3] <= max_age]
        if not valid:
            return None
        candidates.append(valid)
    best, distance = None, (float("inf"), float("inf"))
    for frames in itertools.product(*candidates):
        stamps = [frame[2] for frame in frames]
        if max(stamps)-min(stamps) > camera_tolerance+1e-9:
            continue
        # Extend single-arm synchronized_camera_pair's score to three cameras:
        # minimize worst target error, then camera-to-camera timestamp span.
        error = (max(abs(stamp-target) for stamp in stamps), max(stamps)-min(stamps))
        if error < distance:
            best, distance = dict(zip(cameras, frames)), error
    return best


def vector(msg):
    values = np.asarray(msg.position, dtype=np.float32)
    if values.shape != (7,) or not np.isfinite(values).all():
        raise ValueError("Expected seven finite joint/gripper values")
    stamp = msg.header.stamp.to_sec()
    if not math.isfinite(stamp) or stamp <= 0:
        raise ValueError("A positive header timestamp is required")
    return values


class Buffer(RosVectorBuffer):
    """Reuse single-arm buffering/MCAP helpers with a known message type."""
    def __init__(self, topic, label, mcap, extractor=vector, msg_class=JointState):
        self.topic = self.real_topic = topic
        self.label, self.mcap_recorder = label, mcap
        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)
        self.buffer = deque(maxlen=500)
        self.error_reported = False
        self.extractor, self.msg_class = extractor, msg_class
        self.subscriber = rospy.Subscriber(topic, msg_class, self.callback, queue_size=200)


def synchronized(sources, target, tolerance):
    samples = {key: source.nearest(target) for key, source in sources.items()}
    if any(value is None or value.shape != (7,) or not np.isfinite(value).all()
           or not math.isfinite(stamp) or not math.isfinite(error) or abs(error) > tolerance
           for value, stamp, error in samples.values()):
        return None
    return samples


class Recorder:
    def __init__(self):
        self.config = configuration(rospy.get_param("~configuration", "dual"))
        self.cameras = self.config.cameras
        self.enable_cameras = bool(rospy.get_param("~enable_cameras", True))
        requested_root = Path(rospy.get_param("~dataset_root", str(DATASET_ROOT))).expanduser()
        self.root = requested_root
        self.fps = float(rospy.get_param("~fps", 20))
        self.tolerance = float(rospy.get_param("~sync_tolerance", .03))
        self.wait = float(rospy.get_param("~sync_wait", .025))
        self.max_gap = float(rospy.get_param("~max_data_gap", 1.))
        self.sync_delay = float(rospy.get_param("~sync_delay", .2)) if self.enable_cameras else 0.
        self.camera_target_tolerance = float(rospy.get_param("~camera_target_tolerance", .10))
        self.camera_tolerance = float(rospy.get_param("~camera_sync_tolerance", .03))
        self.camera_wait = float(rospy.get_param("~camera_sync_wait", .10))
        self.max_image_age = float(rospy.get_param("~max_image_age", .5))
        self.jpeg_quality = int(rospy.get_param("~jpeg_quality", 95))
        self.preview_enabled = self.enable_cameras and bool(rospy.get_param("~preview", True))
        self.camera_fps = float(rospy.get_param("~camera_fps", 30))
        self.camera_backend = rospy.get_param("~camera_backend", "v4l2")
        if self.camera_backend not in ("v4l2", "default"):
            raise ValueError("camera_backend must be v4l2 or default")
        self.camera_open_timeout = float(rospy.get_param("~camera_open_timeout", 5.))
        self.camera_read_timeout = float(rospy.get_param("~camera_read_timeout", 2.))
        values = (self.fps, self.tolerance, self.wait, self.max_gap)
        if not all(math.isfinite(v) for v in values) or self.fps <= 0 or self.tolerance < 0 or self.wait < 0 or self.max_gap <= 0:
            raise ValueError("Invalid FPS/synchronization settings")
        camera_values = (self.sync_delay, self.camera_target_tolerance, self.camera_tolerance,
                         self.max_image_age, self.camera_fps, self.camera_open_timeout, self.camera_read_timeout, self.camera_wait)
        if not all(math.isfinite(v) for v in camera_values) or min(camera_values) < 0 or min(
                self.max_image_age, self.camera_fps, self.camera_open_timeout, self.camera_read_timeout) <= 0:
            raise ValueError("Invalid camera timing settings")
        if self.enable_cameras and self.max_image_age < self.sync_delay+self.camera_target_tolerance:
            raise ValueError("max_image_age must cover sync_delay + camera_target_tolerance")
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be 1..100")
        if rospy.get_param("/use_sim_time", False):
            raise ValueError("This hardware recorder requires /use_sim_time=false")
        defaults = {side+"_"+kind: "/follower"+suffix+"/"+leaf
                    for side, suffix in self.config.arms
                    for kind, leaf in (("state", "current_joint_obs"), ("action", "applied_joint_cmd"))}
        self.topics = {key: rospy.get_param("~"+key+"_topic", default) for key, default in defaults.items()}
        self.metadata = {
            "format": "piper_dual_state_mcap_v1", "fps": self.fps,
            "state_dim": self.config.dimension, "action_dim": self.config.dimension, "names": self.config.names,
            "units": {"joints": "rad", "gripper": "m", "timestamps": "seconds"},
            "topics": self.topics, "sync_tolerance": self.tolerance, "sync_wait": self.wait,
            "max_data_gap": self.max_gap, "cameras": [],
            "repo_id": rospy.get_param("~repo_id", "local/piper_dual_state"),
            "task": rospy.get_param("~task", "bimanual demonstration"),
            "task_scope": "episode",
            "timestamp_semantics": "State: existing publication time; Action: SDK dispatch completion time",
            "action_semantics": "SDK-eligible dispatched commands, after gripper clamp and wire quantization; not hardware acknowledgement",
            "sync_error_semantics": "signed message header stamp minus fixed-grid target; sync_span=max(stamps)-min(stamps)",
            "mcap_semantics": "Original four ROS streams during recording; asynchronous, including samples absent from synchronized NPZ",
        }
        self.camera_configs = {}
        if self.enable_cameras:
            for name, device, width, height in self.config.camera_defaults:
                self.camera_configs[name] = {
                    "device": rospy.get_param("~"+name+"_camera", device),
                    "width": int(rospy.get_param("~"+name+"_width", width)),
                    "height": int(rospy.get_param("~"+name+"_height", height)),
                    "fps": self.camera_fps, "fourcc": rospy.get_param("~camera_fourcc", "MJPG"),
                }
            if len({cfg["device"] for cfg in self.camera_configs.values()}) != len(self.cameras):
                raise ValueError("Distinct camera devices required")
            if any(min(cfg["width"], cfg["height"]) <= 0 or len(cfg["fourcc"]) != 4 for cfg in self.camera_configs.values()):
                raise ValueError("Invalid camera size or FourCC")
            self.metadata.update(format="piper_dual_state_mcap_v2", cameras=list(self.cameras),
                camera_requested=self.camera_configs, sync_delay=self.sync_delay,
                camera_backend=self.camera_backend,
                camera_target_tolerance=self.camera_target_tolerance, camera_sync_tolerance=self.camera_tolerance,
                camera_sync_wait=self.camera_wait,
                max_image_age=self.max_image_age, jpeg_quality=self.jpeg_quality,
                image_semantics="Native JPEG frames; reuse allowed within tolerance; minimize worst grid-time error then camera span",
                reuse_count_semantics="Per camera: equality with preceding saved frame sequence; count extra uses",
                camera_timestamp_semantics="V4L2 frame timestamp mapped to Unix when valid; otherwise read midpoint",
                mcap_semantics="Original four ROS streams plus synchronized compressed images with capture timestamps")
        self.config.configure_metadata(self.metadata)
        self.root.mkdir(parents=True, exist_ok=True)
        metadata_path = self.root/"dataset_meta.json"
        if not metadata_path.exists():
            atomic_json(metadata_path, self.metadata)
        # Preserve all existing data, including root metadata. Episode metadata
        # contains the authoritative task/schema/settings for each recording.
        self.dataset_count = len(saved_episode_folders(self.root))
        rospy.loginfo("Dataset root: %s; Dataset Count: %d", self.root, self.dataset_count)
        self.lock = threading.RLock()
        self.closing = threading.Event()
        self.episode = None
        self.closed = False
        self.last_result = "Ready"
        self.current_task = str(self.metadata["task"])
        self.task_input_active = False
        self.task_input = ""
        self.camera_capture = None
        self.camera_error = None
        self.camera_settings = {}
        self.last_camera_sequence = dict.fromkeys(self.cameras, -1)
        self.key_pool = ThreadPoolExecutor(max_workers=1)
        self.key_future = None
        self.mcap = McapEpisodeRecorder()
        self.sources = {key: Buffer(topic, key, self.mcap) for key, topic in self.topics.items()}
        self.config.add_sources(self)
        for name in ("start", "stop", "discard"):
            rospy.Service("~"+name, Trigger, self.service(name))
        # A ROS signal may interrupt step() on the main thread. Finalize in
        # main's finally block instead of closing files in the signal hook.
        rospy.on_shutdown(self.closing.set)
        if self.enable_cameras:
            try:
                config = dict(fps=self.camera_fps, backend=self.camera_backend,
                    cameras=self.camera_configs, open_timeout=self.camera_open_timeout,
                    read_timeout=self.camera_read_timeout)
                devices = [(name, self.camera_configs[name]["device"]) for name in self.cameras]
                self.camera_capture = CameraCapture(devices, config, worker=capture_worker)
                self.camera_capture.wait_ready((self.camera_open_timeout+10)*len(self.cameras)+2,
                    lambda: self.closing.is_set() or rospy.is_shutdown())
                self.refresh_camera_settings()
            except Exception:
                self.shutdown()
                raise

    def refresh_camera_settings(self):
        settings = json.loads(json.dumps(self.camera_capture.settings))
        for name, actual in settings.items():
            if actual == self.camera_settings.get(name):
                continue
            requested = self.camera_configs[name]
            rospy.loginfo("Camera %s: %dx%d @ %.2f FPS FourCC=%s", actual["device"],
                          actual["width"], actual["height"], actual["fps"], actual["fourcc"])
            if any(actual[key] != requested[key] for key in ("width", "height", "fourcc")) or abs(actual["fps"]-requested["fps"]) > .5:
                rospy.logwarn("Camera %s actual settings differ: requested=%s actual=%s", name, requested, actual)
        self.camera_settings = settings

    def image_snapshots(self):
        if self.camera_error:
            raise RuntimeError(self.camera_error)
        return {name: self.camera_capture.views[name].frame_snapshot() for name in self.cameras}

    def select_images(self, target):
        deadline = time.monotonic()+self.camera_wait
        while not self.closing.is_set() and not rospy.is_shutdown():
            images = synchronized_images(self.image_snapshots(), target, rospy.Time.now().to_sec(),
                time.monotonic(), self.last_camera_sequence, self.camera_target_tolerance,
                self.camera_tolerance, self.max_image_age, self.cameras)
            if images is not None:
                return images
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                return None
            self.closing.wait(min(.002, remaining))
        return None

    def service(self, name, task_snapshot=None):
        def invoke(_):
            try:
                result = self.start(task_snapshot) if name == "start" else getattr(self, name)()
                self.last_result = result
                return TriggerResponse(True, result)
            except Exception as exc:
                self.last_result = str(exc)
                rospy.logerr("Recorder %s: %s", name, exc)
                return TriggerResponse(False, str(exc))
        return invoke

    def start(self, task_snapshot=None):
        with self.lock:
            if self.closed or self.closing.is_set():
                raise RuntimeError("Recorder is closing")
            if self.episode:
                raise RuntimeError("Already recording")
            if self.task_input_active:
                raise RuntimeError("Confirm task input with Enter before recording")
            anchor = rospy.Time.now().to_sec()
            # Do not fabricate State or Action before actual dispatch exists.
            if synchronized(self.sources, anchor, self.tolerance) is None:
                raise RuntimeError("Fresh State/Action required; check following and topics")
            if self.enable_cameras:
                snapshots = self.image_snapshots()
                if any(not frames or time.monotonic()-frames[-1][3] > self.max_image_age for frames in snapshots.values()):
                    raise RuntimeError("Fresh images from all cameras required")
            task = self.current_task if task_snapshot is None else task_snapshot
            episode = self.config.episode(self.root, dict(self.metadata, task=task), anchor)
            try:
                self.mcap.start(episode.path)
            except Exception:
                self.mcap.discard()
                episode.csv.close()
                shutil.rmtree(str(episode.path))
                raise
            self.episode = episode
            self.next_index = 0
            self.last_ros_time = anchor
            self.last_valid = time.monotonic()
            self.last_camera_sequence = dict.fromkeys(self.cameras, -1)
            return "Recording started: "+str(episode.path)

    def _finish(self, status, reason=""):
        episode = self.episode
        if episode is None:
            return "Not recording"
        if status == "saved" and not episode.rows:
            status, reason = "incomplete", "No synchronized rows"
        # Stop MCAP callbacks and close its footer before publishing dataset.
        if status == "saved":
            self.mcap.finish()
        else:
            self.mcap.finish_incomplete()
        if self.enable_cameras:
            episode.metadata["camera_actual"] = dict(self.camera_settings)
            episode.metadata["camera_measured_fps"] = {
                name: self.camera_capture.views[name].measured_fps() for name in self.cameras}
        path = episode.finish(status, reason)
        self.episode = None
        if status == "saved":
            self.dataset_count = len(saved_episode_folders(self.root))
        return status+": "+str(path)

    def stop(self):
        with self.lock:
            return self._finish("saved")

    def discard(self):
        with self.lock:
            if self.episode is None:
                return "Not recording"
            self.mcap.discard()
            self.episode.csv.close()
            shutil.rmtree(str(self.episode.path))
            self.episode = None
            return "Episode discarded"

    def step(self):
        with self.lock:
            snapshots = None
            if self.enable_cameras and not self.closed:
                try:
                    snapshots = self.image_snapshots()
                    self.refresh_camera_settings()
                except Exception as exc:
                    self.camera_error = str(exc)
                    if self.episode:
                        rospy.logerr(self._finish("incomplete", "Camera capture failed: "+str(exc)))
                    return
            if self.episode is None or self.closed or self.closing.is_set():
                return
            now = rospy.Time.now().to_sec()
            if now < self.last_ros_time:
                rospy.logerr(self._finish("incomplete", "ROS clock moved backwards"))
                return
            self.last_ros_time = now
            index = int(math.floor((now-self.episode.anchor-self.sync_delay)*self.fps+1e-6))
            if index < self.next_index:
                return
            self.episode.dropped["schedule"] += index-self.next_index
            target = self.episode.anchor+index/self.fps
            images = None
            if self.enable_cameras:
                images = self.select_images(target)
            deadline = time.monotonic()+self.wait
            for source in self.sources.values():
                source.wait_until_at_least(target, deadline)
            if self.closing.is_set():
                return
            samples = synchronized(self.sources, target, self.tolerance)
            self.next_index = index+1
            if samples is None or self.enable_cameras and images is None:
                self.episode.dropped["missing_or_unsynced"] += 1
                if time.monotonic()-self.last_valid > self.max_gap:
                    rospy.logerr(self._finish("incomplete", "State/Action/images missing or unsynchronized"))
            else:
                paths = None
                if self.enable_cameras:
                    try:
                        paths = self.save_images(images)
                    except Exception as exc:
                        rospy.logerr(self._finish("incomplete", "Image save failed: "+str(exc)))
                        return
                self.episode.add(target, samples, images, paths)
                if images:
                    self.last_camera_sequence = {name: images[name][0] for name in self.cameras}
                self.last_valid = time.monotonic()

    def save_images(self, images):
        encoded = {}
        for name in self.cameras:
            _, frame, stamp, _ = images[name]
            encoded[name] = make_compressed_image(frame, rospy.Time.from_sec(stamp), name, self.jpeg_quality)
        paths = {}
        for name, (msg, data) in encoded.items():
            relative = name+"/%06d.jpg" % len(self.episode.rows)
            if hasattr(self.episode, "stage_image"):
                self.episode.stage_image(name, data)
            else:
                destination = self.episode.path/relative
                temporary = destination.with_suffix(".jpg.partial")
                temporary.write_bytes(data)
                os.replace(str(temporary), str(destination))
            self.mcap.write_ros_message("/camera/"+name+"/compressed", msg)
            paths[name] = relative
        return paths

    def begin_task_input(self):
        with self.lock:
            if self.closed or self.closing.is_set():
                return False
            if self.episode is not None:
                rospy.logwarn("Cannot change task while recording")
                return False
            if self.key_future is not None and not self.key_future.done():
                rospy.logwarn("Wait for the recorder operation before editing task")
                return False
            self.task_input_active = True
            self.task_input = ""
            rospy.loginfo("Current task: %s", self.current_task)
            rospy.loginfo("Type task in PiPER-Recorder terminal; Enter=confirm, empty=keep, Esc=cancel")
            self.render_task_input()
            return True

    def render_task_input(self):
        # The existing xterm is the Unicode input surface; keep camera GUI live.
        sys.stdout.write("\r\033[2KTask input: "+self.task_input)
        sys.stdout.flush()

    def handle_task_input(self, key):
        with self.lock:
            if not self.task_input_active or self.episode is not None:
                return
            if key in ("\r", "\n", "\x1b"):
                if key != "\x1b" and self.task_input.strip():
                    self.current_task = self.task_input.strip()
                self.task_input_active = False
                self.task_input = ""
                sys.stdout.write("\n")
                sys.stdout.flush()
                rospy.loginfo("Current task: %s", self.current_task)
            elif key in ("\x08", "\x7f"):
                self.task_input = self.task_input[:-1]
                self.render_task_input()
            elif key.isprintable():
                self.task_input += key
                self.render_task_input()

    def handle_key(self, key, source="terminal"):
        if self.task_input_active:
            # OpenCV waitKey cannot reliably enter Japanese text. While typing,
            # route text from the existing terminal and keep previews running.
            if source == "terminal":
                self.handle_task_input(key)
            return True
        if key == "t":
            self.begin_task_input()
            return True
        if key == "q":
            return False
        if key in ("r", "s", "x"):
            if self.key_future is None or self.key_future.done():
                task_snapshot = self.current_task
                def operate():
                    result = self.service({"r": "start", "s": "stop", "x": "discard"}[key], task_snapshot)(None)
                    (rospy.loginfo if result.success else rospy.logerr)(result.message)
                self.key_future = self.key_pool.submit(operate)
            else:
                rospy.logwarn("Recorder operation still running")
        return True

    def preview(self):
        if not self.preview_enabled:
            return -1
        tiles = []
        try:
            snapshots = self.image_snapshots()
        except Exception:
            snapshots = {name: [] for name in self.cameras}
        # Copy only for display; stored images preserve their native resolution.
        for name in self.cameras:
            frames = snapshots[name]
            frame = cv2.resize(frames[-1][1], (480, 270)) if frames else np.zeros((270, 480, 3), np.uint8)
            stale = not frames or time.monotonic()-frames[-1][3] > self.max_image_age
            label = name.upper()+(" STALE / MISSING" if stale else "")
            cv2.putText(frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, .7,
                        (0, 0, 255) if stale else (255, 255, 255), 2)
            tiles.append(frame)
        combined = np.concatenate(tiles, axis=1)
        cv2.putText(combined, "Dataset Count: %d" % self.dataset_count, (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2)
        active = self.episode is not None
        label = "RECORDING" if active else "IDLE"
        if self.camera_error:
            label = "CAMERA ERROR"
        if self.task_input_active:
            label += " | TASK INPUT IN TERMINAL (Enter=confirm)"
        cv2.putText(combined, label+" | t=task r=start s=save x=discard q=quit", (10, 258),
                    cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 0, 255) if active or self.camera_error else (0, 255, 0), 2)
        combined = draw_details(combined, self)
        cv2.imshow("PiPER " + self.config.name + " teaching recorder", combined)
        return cv2.waitKey(1) & 0xFF

    def sampling_loop(self):
        try:
            while not self.closing.is_set() and not rospy.is_shutdown():
                self.step()
                self.closing.wait(.002)
        except Exception as exc:
            rospy.logerr("Recording failed: %s", exc)
            self.closing.set()

    def shutdown(self):
        self.closing.set()
        with self.lock:
            if self.closed:
                return
            self.closed = True
            try:
                if self.episode:
                    rospy.loginfo(self._finish("incomplete", "Recorder shutdown"))
            finally:
                if self.camera_capture:
                    self.camera_capture.close()
                self.key_pool.shutdown(wait=False)


def main():
    rospy.init_node("unified_state_mcap_recorder")
    cv2.setNumThreads(1)
    recorder = Recorder()
    sampling = threading.Thread(target=recorder.sampling_loop, daemon=True, name="dual_recording_writer")
    sampling.start()
    terminal = attributes = None
    try:
        try:
            terminal = open("/dev/tty", "r")
            attributes = termios.tcgetattr(terminal.fileno())
            tty.setcbreak(terminal.fileno())
        except (OSError, termios.error):
            if terminal:
                terminal.close()
            terminal = None
        rospy.loginfo("Recorder only: t=task r=start s=save x=discard q=quit. No robot motion/mode changes.")
        rospy.loginfo("Current task: %s", recorder.current_task)
        decoder = codecs.getincrementaldecoder("utf-8")("replace")
        if terminal is None:
            rospy.loginfo("No terminal; use ~start / ~stop / ~discard Trigger services")
        while not rospy.is_shutdown() and not recorder.closing.is_set():
            key = recorder.preview()
            if key != -1 and key != 255 and not recorder.handle_key(chr(key), source="preview"):
                break
            if terminal and select.select([terminal], [], [], 0)[0]:
                text = decoder.decode(os.read(terminal.fileno(), 4096))
                if any(not recorder.handle_key(key) for key in text):
                    break
            time.sleep(1./30.)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        rospy.logerr("Recorder failed: %s", exc)
        raise
    finally:
        try:
            recorder.closing.set()
            sampling.join(timeout=1.)
            recorder.shutdown()
        finally:
            if recorder.preview_enabled:
                cv2.destroyAllWindows()
            if terminal:
                if attributes is not None:
                    termios.tcsetattr(terminal.fileno(), termios.TCSANOW, attributes)
                terminal.close()


if __name__ == "__main__":
    main()
