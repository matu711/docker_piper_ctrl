"""Sequential camera capture with a separate owner process and bounded cleanup."""
from collections import deque
import math
import multiprocessing
import os
import signal
import sys
import threading
import time

import cv2


def camera_diagnostic(message):
    # The spawned capture process has no ROS node. Flush to roslaunch's
    # captured stderr, including a read-start marker if a native read hangs.
    try:
        print("[mcap2_camera] "+message, file=sys.stderr, flush=True)
    except OSError:
        # A closed diagnostic stream must not become a camera/control fault.
        pass


class CaptureDiagnostics:
    """Log the first three batches, then rate-limit anomalies per camera."""
    def __init__(self, timeout):
        self.timeout = timeout
        self.last_log = {}
        self.previous = {}

    def emit(self, name, message, startup=False):
        now = time.monotonic()
        key = (name, message.split()[0])
        if startup or now-self.last_log.get(key, float("-inf")) >= 2.:
            camera_diagnostic(message)
            self.last_log[key] = now

    def read(self, cap, name, device, sequence, attempt):
        startup = sequence < 3
        before_ros, before_mono = time.time(), time.monotonic()
        if startup:
            self.emit(name, "read_start camera=%s device=%s sequence=%d attempt=%d "
                      "unix=%.6f mono=%.6f" %
                      (name, device, sequence, attempt, before_ros, before_mono), startup=True)
            # Keep diagnostic output time outside the read measurement and
            # timestamp fallback. read_end carries these exact read bounds.
            before_ros, before_mono = time.time(), time.monotonic()
        try:
            ok, frame = cap.read()
        except Exception as exc:
            after_mono, after_ros = time.monotonic(), time.time()
            self.emit(name, "read_exception camera=%s device=%s sequence=%d attempt=%d "
                      "start_unix=%.6f end_unix=%.6f start_mono=%.6f end_mono=%.6f "
                      "duration=%.6fs error=%s" %
                      (name, device, sequence, attempt, before_ros, after_ros,
                       before_mono, after_mono, after_mono-before_mono, exc), startup)
            raise
        after_mono, after_ros = time.monotonic(), time.time()
        success = bool(ok and frame is not None)
        if startup or not success or after_mono-before_mono > self.timeout:
            self.emit(name, "read_end camera=%s device=%s sequence=%d attempt=%d "
                      "start_unix=%.6f end_unix=%.6f start_mono=%.6f end_mono=%.6f "
                      "duration=%.6fs success=%s first_frame=%s" %
                      (name, device, sequence, attempt, before_ros, after_ros, before_mono,
                       after_mono, after_mono-before_mono, success, sequence == 0), startup)
        return ok, frame, before_ros, before_mono, after_ros, after_mono

    def frame(self, name, device, sequence, stamp, mono, source, read_end):
        previous = self.previous.get(name)
        interval = read_end-previous[0] if previous else None
        changed = previous is not None and source != previous[1]
        if sequence < 3 or changed or (interval is not None and interval > self.timeout):
            self.emit(name, "frame camera=%s device=%s sequence=%d first_frame=%s "
                      "stamp_unix=%.6f stamp_mono=%.6f source=%s "
                      "age_at_read_end=%.6fs update_interval=%s" %
                      (name, device, sequence, sequence == 0, stamp, mono, source,
                       read_end-mono, "none" if interval is None else "%.6fs" % interval), sequence < 3)
        self.previous[name] = (read_end, source)


def frame_time(cap, before_ros, before_mono, after_ros, after_mono, age_limit):
    """Prefer the V4L2 dequeued-frame timestamp; validate its clock domain."""
    try:
        stamp = float(cap.get(cv2.CAP_PROP_POS_MSEC))/1000.
    except (TypeError, ValueError, cv2.error):
        stamp = 0.
    if math.isfinite(stamp) and stamp > 0:
        offset = ((before_ros-before_mono)+(after_ros-after_mono))/2
        if 0 <= after_mono-stamp <= age_limit:
            return stamp+offset, stamp, "v4l2_monotonic"
        if 0 <= after_ros-stamp <= age_limit:
            return stamp, stamp-offset, "v4l2_realtime"
    return (before_ros+after_ros)/2, (before_mono+after_mono)/2, "read_midpoint_fallback"


def _open(device, config, stop):
    """Match test_dual_camera: configure without reading warm-up frames."""
    deadline = time.monotonic()+config["open_timeout"]
    while not stop.is_set():
        backend = cv2.CAP_V4L2 if config["backend"] == "v4l2" else cv2.CAP_ANY
        cap = cv2.VideoCapture(device, backend)
        try:
            if not cap.isOpened():
                raise RuntimeError("Could not open camera: "+device)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*config["fourcc"]))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, config["width"])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config["height"])
            cap.set(cv2.CAP_PROP_FPS, config["fps"])
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            code = int(cap.get(cv2.CAP_PROP_FOURCC))
            settings = {"device": device, "width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)), "fps": cap.get(cv2.CAP_PROP_FPS),
                "fourcc": "".join(chr((code >> (8*i)) & 255) for i in range(4)),
                "output_width": config["width"], "output_height": config["height"]}
            return cap, settings
        except Exception:
            cap.release()
            if time.monotonic() >= deadline:
                raise
            stop.wait(.2)
    raise RuntimeError("Camera startup cancelled")


def _camera_worker(devices, config, stop, connection):
    # The recorder owns SIGINT and file finalization. A native V4L2 read can
    # ignore Python signals, so its owner can be terminated independently.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    # Match the working preview's default native wait. Stream-on can take
    # several seconds; a one-second select caused false startup failures.
    # Parent cleanup remains bounded by this separate process's lifetime.
    os.environ["OPENCV_VIDEOIO_V4L_SELECT_TIMEOUT"] = "10"
    cv2.setNumThreads(1)
    cameras = []
    diagnostics = CaptureDiagnostics(config["read_timeout"])
    try:
        settings = {}
        for name, device in devices:
            cap, settings[name] = _open(device, config, stop)
            cameras.append((name, device, cap))
        connection.send(("settings", settings))
        sequence = 0
        while not stop.is_set():
            started = time.monotonic()
            batch = {}
            # Exactly the triple-camera test's single-owner, ordered reads.
            for index, (name, device, cap) in enumerate(cameras):
                if stop.is_set():
                    return
                ok, frame, before_ros, before_mono, after_ros, after_mono = diagnostics.read(
                    cap, name, device, sequence, 0)
                # Some UVC devices produce no first frame on initial stream-on.
                # Retry on the owner process; never race release() against read().
                if sequence == 0:
                    deadline = time.monotonic()+config["open_timeout"]
                    failures = 0
                    while not ok or frame is None:
                        if ok and frame is not None or stop.is_set():
                            break
                        if time.monotonic() >= deadline:
                            break
                        failures += 1
                        if failures % 2 == 0:
                            # Repeated reads of a stuck initial stream do not
                            # requeue/reset it. Reopen exclusively on its owner.
                            cap.release()
                            remaining = dict(config, open_timeout=max(.01, deadline-time.monotonic()))
                            cap, settings[name] = _open(device, remaining, stop)
                            cameras[index] = (name, device, cap)
                            settings[name]["startup_reopens"] = failures//2
                        stop.wait(.1)
                        ok, frame, before_ros, before_mono, after_ros, after_mono = diagnostics.read(
                            cap, name, device, sequence, failures)
                if stop.is_set():
                    return
                if not ok or frame is None:
                    raise RuntimeError("Could not read a frame from "+device)
                stamp, mono, source = frame_time(cap, before_ros, before_mono, after_ros, after_mono,
                                                 config["read_timeout"])
                settings[name]["timestamp_source"] = source
                diagnostics.frame(name, device, sequence, stamp, mono, source, after_mono)
                if frame.shape[1] != config["width"] or frame.shape[0] != config["height"]:
                    frame = cv2.resize(frame, (config["width"], config["height"]))
                batch[name] = (sequence, frame, stamp, mono)
            if sequence == 0:
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
        # VideoCapture is accessed and released exclusively in this process.
        for _, _, cap in cameras:
            cap.release()
        connection.close()


class CameraView:
    def __init__(self, owner, name):
        self.owner, self.name = owner, name

    def frame_snapshot(self):
        with self.owner.condition:
            self.owner.check()
            return list(self.owner.frames[self.name])

    def measured_fps(self):
        with self.owner.condition:
            stamps = list(self.owner.frame_times[self.name])
            return (len(stamps)-1)/(stamps[-1]-stamps[0]) if len(stamps)>1 and stamps[-1]>stamps[0] else 0.


class CameraCapture:
    def __init__(self, devices, config, worker=_camera_worker):
        self.config = config
        self.settings, self.error = {}, None
        self.condition = threading.Condition()
        self.frames = {name: deque(maxlen=8) for name, _ in devices}
        self.frame_times = {name: deque(maxlen=120) for name, _ in devices}
        self.views = {name: CameraView(self, name) for name, _ in devices}
        self.last_receive = None
        self.closed = False
        context = multiprocessing.get_context("spawn")
        self.connection, sender = context.Pipe(duplex=False)
        self.stop_event = context.Event()
        self.process = context.Process(target=worker, args=(devices, config, self.stop_event, sender),
                                       name="mcap2_camera_capture", daemon=True)
        try:
            self.process.start()
        except Exception:
            sender.close()
            self.connection.close()
            raise
        sender.close()
        self.receiver = threading.Thread(target=self._receive, daemon=True, name="mcap2_camera_receiver")
        self.receiver.start()

    def _receive(self):
        try:
            while True:
                if not self.connection.poll(.1):
                    if self.closed and not self.process.is_alive():
                        return
                    continue
                kind, data = self.connection.recv()
                with self.condition:
                    if kind == "settings":
                        self.settings = data
                    elif kind == "error":
                        self.error = RuntimeError(data)
                    elif kind == "frames":
                        if not self.closed:
                            for name, frame in data.items():
                                self.frames[name].append(frame)
                                self.frame_times[name].append(frame[3])
                            self.last_receive = time.monotonic()
                    self.condition.notify_all()
        except (EOFError, OSError) as exc:
            with self.condition:
                if not self.closed and self.error is None:
                    self.error = RuntimeError("Camera capture process exited: "+str(exc))
                self.condition.notify_all()

    def check(self):
        if self.error:
            raise self.error
        if self.closed:
            raise RuntimeError("Camera capture closed")
        if self.last_receive is not None and time.monotonic()-self.last_receive > self.config["read_timeout"]:
            raise RuntimeError("Camera read timed out; inspect USB connection/competing processes")

    def wait_ready(self, timeout, cancelled=lambda: False):
        deadline = time.monotonic()+timeout
        with self.condition:
            while not all(self.frames.values()):
                self.check()
                if cancelled():
                    raise RuntimeError("Camera startup cancelled")
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    raise RuntimeError("Timed out waiting for the first three camera frames")
                self.condition.wait(min(.1, remaining))
            self.check()

            now = time.monotonic()
            for name, frames in self.frames.items():
                latest = frames[-1]
                camera_diagnostic("wait_ready camera=%s sequence=%d age=%.6fs threshold=%.6fs "
                                  "stamp_unix=%.6f stamp_mono=%.6f source=%s" %
                                  (name, latest[0], now-latest[3], self.config["read_timeout"],
                                   latest[2], latest[3], self.settings.get(name, {}).get("timestamp_source", "unknown")))

    def close(self):
        if self.closed:
            return False
        self.closed = True
        self.stop_event.set()
        with self.condition:
            self.condition.notify_all()
        self.process.join(timeout=.75)
        forced = self.process.is_alive()
        if forced:
            self.process.terminate()
            self.process.join(timeout=.75)
        if self.process.is_alive():
            self.process.kill()
            self.process.join(timeout=.5)
        self.receiver.join(timeout=.25)
        self.connection.close()
        if not self.process.is_alive() and not self.receiver.is_alive():
            self.process.close()
        return forced
