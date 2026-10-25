#!/usr/bin/env python3
"""Publish two VLA observations from independent camera capture threads.

Only the most recent frame from each camera is retained. Capture never waits
for JPEG encoding or ROS publishing, so one camera cannot block the other.
"""

import threading
import time

import cv2
import rospy
from sensor_msgs.msg import CompressedImage


def fourcc_string(value):
    value = int(value)
    return "".join(chr((value >> (8 * index)) & 0xFF) for index in range(4))


def validate_camera_config(label, fourcc, capture_fps):
    if len(fourcc) != 4:
        raise ValueError("%s_fourcc must contain exactly four characters" % label)
    if capture_fps <= 0:
        raise ValueError("%s_capture_fps must be positive" % label)


class LatestFrame:
    """A one-element overwrite-only frame store."""

    def __init__(self):
        self._lock = threading.Lock()
        self._frame = None
        self._stamp = None
        self._capture_monotonic = None

    def put(self, frame, stamp, capture_monotonic):
        # cap.read() has already completed.  This short critical section only
        # replaces the one retained frame; it never calls into OpenCV.
        with self._lock:
            self._frame = frame
            self._stamp = stamp
            self._capture_monotonic = capture_monotonic

    def get_copy(self):
        """Return an owned image copy; JPEG encoding happens after unlock."""
        with self._lock:
            if self._frame is None:
                return None, None, None
            return self._frame.copy(), self._stamp, self._capture_monotonic


class CameraCaptureWorker(threading.Thread):
    """Capture one V4L2 device continuously without accumulating frames."""

    def __init__(
        self, label, device, backend, width, height, capture_fps, fourcc,
        latest_frame, stop_event, debug_stats, diagnostic_period_s,
    ):
        super().__init__(name="%s_camera_capture" % label)
        self.daemon = True
        self.label = label
        self.device = device
        self.backend = backend
        self.width = width
        self.height = height
        self.capture_fps = capture_fps
        self.fourcc = fourcc
        self.latest_frame = latest_frame
        self.stop_event = stop_event
        self.debug_stats = debug_stats
        self.diagnostic_period_s = diagnostic_period_s
        self.ready_event = threading.Event()
        self.error = None
        self.cap = None

    def _open(self):
        self.cap = cv2.VideoCapture(self.device, self.backend)
        if not self.cap.isOpened():
            raise RuntimeError("Could not open %s camera: %s" % (self.label, self.device))
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.capture_fps)

        actual_fourcc = fourcc_string(self.cap.get(cv2.CAP_PROP_FOURCC))
        actual_width = int(round(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
        actual_height = int(round(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        rospy.loginfo(
            "%s camera: device=%s requested=%s %dx%d @%.2f actual=%s %dx%d @%.2f",
            self.label.capitalize(), self.device, self.fourcc, self.width,
            self.height, self.capture_fps, actual_fourcc, actual_width,
            actual_height, actual_fps,
        )
        if (
            actual_fourcc != self.fourcc
            or actual_width != self.width
            or actual_height != self.height
            or abs(actual_fps - self.capture_fps) > 0.5
        ):
            rospy.logwarn(
                "%s camera negotiated a different mode: requested=%s %dx%d @%.2f "
                "actual=%s %dx%d @%.2f",
                self.label.capitalize(), self.fourcc, self.width, self.height,
                self.capture_fps, actual_fourcc, actual_width, actual_height,
                actual_fps,
            )

    def run(self):
        read_count = 0
        read_total_s = 0.0
        report_started = time.monotonic()
        try:
            self._open()
            self.ready_event.set()
            while not self.stop_event.is_set() and not rospy.is_shutdown():
                # This timing intentionally contains only cap.read(), not a
                # lock wait, frame copy, ROS call, or any sleep.
                read_started = time.perf_counter()
                ok, frame = self.cap.read()
                read_total_s += time.perf_counter() - read_started
                read_count += 1
                if ok and frame is not None:
                    self.latest_frame.put(
                        frame, rospy.Time.now(), time.monotonic()
                    )
                else:
                    rospy.logwarn_throttle(
                        1.0, "Failed to read %s camera: %s", self.label, self.device
                    )

                elapsed = time.monotonic() - report_started
                if self.debug_stats and elapsed >= self.diagnostic_period_s:
                    rospy.loginfo(
                        "%s capture stats: reads=%d avg_read=%.1f ms effective=%.2f Hz",
                        self.label.capitalize(), read_count,
                        (read_total_s / max(read_count, 1)) * 1000.0,
                        read_count / elapsed,
                    )
                    read_count = 0
                    read_total_s = 0.0
                    report_started = time.monotonic()
        except Exception as error:
            self.error = error
            rospy.logerr("%s camera worker failed: %s", self.label.capitalize(), error)
            self.ready_event.set()
        finally:
            if self.cap is not None:
                self.cap.release()


def make_message(frame, stamp, frame_id, quality):
    encode_started = time.perf_counter()
    encoded_ok, encoded = cv2.imencode(
        ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality]
    )
    encode_s = time.perf_counter() - encode_started
    if not encoded_ok:
        raise RuntimeError("JPEG encoding failed for %s" % frame_id)
    message = CompressedImage()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.format = "jpeg"
    message.data = encoded.tobytes()
    return message, encode_s


def wait_for_workers(workers):
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline and not rospy.is_shutdown():
        errors = [worker.error for worker in workers if worker.error is not None]
        if errors:
            raise errors[0]
        if all(worker.ready_event.is_set() for worker in workers):
            return
        time.sleep(0.05)
    raise RuntimeError("Timed out while opening VLA cameras")


def main():
    rospy.init_node("dual_compressed_camera_publisher")
    front_device = rospy.get_param("~front_camera", "/dev/video6")
    wrist_device = rospy.get_param("~wrist_camera", "/dev/video0")
    front_fourcc = rospy.get_param("~front_fourcc", "YUYV")
    wrist_fourcc = rospy.get_param("~wrist_fourcc", "MJPG")
    front_capture_fps = float(rospy.get_param("~front_capture_fps", 30))
    wrist_capture_fps = float(rospy.get_param("~wrist_capture_fps", 30))
    publish_fps = float(rospy.get_param("~publish_fps", 20))
    width = int(rospy.get_param("~width", 640))
    height = int(rospy.get_param("~height", 480))
    jpeg_quality = int(rospy.get_param("~jpeg_quality", 90))
    backend_name = rospy.get_param("~backend", "v4l2")
    debug_stats = bool(rospy.get_param("~camera_debug_stats", False))
    diagnostic_period_s = float(rospy.get_param("~diagnostic_period_s", 5.0))

    if width <= 0 or height <= 0 or publish_fps <= 0:
        raise ValueError("width, height, and publish_fps must be positive")
    if not 1 <= jpeg_quality <= 100:
        raise ValueError("jpeg_quality must be in [1, 100]")
    if backend_name not in ("v4l2", "default"):
        raise ValueError("backend must be v4l2 or default")
    if diagnostic_period_s <= 0:
        raise ValueError("diagnostic_period_s must be positive")
    validate_camera_config("front", front_fourcc, front_capture_fps)
    validate_camera_config("wrist", wrist_fourcc, wrist_capture_fps)

    backend = cv2.CAP_V4L2 if backend_name == "v4l2" else cv2.CAP_ANY
    stop_event = threading.Event()
    front_latest = LatestFrame()
    wrist_latest = LatestFrame()
    workers = [
        CameraCaptureWorker(
            "front", front_device, backend, width, height, front_capture_fps,
            front_fourcc, front_latest, stop_event, debug_stats,
            diagnostic_period_s,
        ),
        CameraCaptureWorker(
            "wrist", wrist_device, backend, width, height, wrist_capture_fps,
            wrist_fourcc, wrist_latest, stop_event, debug_stats,
            diagnostic_period_s,
        ),
    ]
    front_pub = rospy.Publisher(
        "/camera/front/compressed", CompressedImage, queue_size=1
    )
    wrist_pub = rospy.Publisher(
        "/camera/wrist/compressed", CompressedImage, queue_size=1
    )

    for worker in workers:
        worker.start()
    try:
        wait_for_workers(workers)
        rospy.loginfo("ROS publish target=%.2f Hz", publish_fps)
        rate = rospy.Rate(publish_fps)
        report_started = time.monotonic()
        publish_count = 0
        front_encode_s = wrist_encode_s = 0.0
        front_ros_publish_s = wrist_ros_publish_s = publish_work_s = 0.0
        while not rospy.is_shutdown():
            work_started = time.monotonic()
            # Each camera has its own LatestFrame lock.  The copy is made while
            # that lock is held, then both JPEG encoding and ROS publishing are
            # performed with no capture lock held.
            front_frame, front_stamp, front_capture_time = front_latest.get_copy()
            wrist_frame, wrist_stamp, wrist_capture_time = wrist_latest.get_copy()
            now = time.monotonic()
            if front_frame is None:
                rospy.logwarn_throttle(1.0, "No front camera frame available yet")
            else:
                message, encode_s = make_message(
                    front_frame, front_stamp, "front_camera", jpeg_quality
                )
                ros_publish_started = time.perf_counter()
                front_pub.publish(message)
                front_ros_publish_s += time.perf_counter() - ros_publish_started
                front_encode_s += encode_s
                if now - front_capture_time > 0.25:
                    rospy.logwarn_throttle(1.0, "Front camera frame is %.0f ms old", (now - front_capture_time) * 1000.0)
            if wrist_frame is None:
                rospy.logwarn_throttle(1.0, "No wrist camera frame available yet")
            else:
                message, encode_s = make_message(
                    wrist_frame, wrist_stamp, "wrist_camera", jpeg_quality
                )
                ros_publish_started = time.perf_counter()
                wrist_pub.publish(message)
                wrist_ros_publish_s += time.perf_counter() - ros_publish_started
                wrist_encode_s += encode_s
                if now - wrist_capture_time > 0.25:
                    rospy.logwarn_throttle(1.0, "Wrist camera frame is %.0f ms old", (now - wrist_capture_time) * 1000.0)
            publish_work_s += time.monotonic() - work_started
            publish_count += 1

            elapsed = time.monotonic() - report_started
            if debug_stats and elapsed >= diagnostic_period_s:
                rospy.loginfo(
                    "Camera publish stats: cycles=%d effective=%.2f Hz "
                    "front_encode=%.1f ms wrist_encode=%.1f ms "
                    "front_ros_publish=%.1f ms wrist_ros_publish=%.1f ms "
                    "publish_work=%.1f ms",
                    publish_count, publish_count / elapsed,
                    front_encode_s / max(publish_count, 1) * 1000.0,
                    wrist_encode_s / max(publish_count, 1) * 1000.0,
                    front_ros_publish_s / max(publish_count, 1) * 1000.0,
                    wrist_ros_publish_s / max(publish_count, 1) * 1000.0,
                    publish_work_s / max(publish_count, 1) * 1000.0,
                )
                report_started = time.monotonic()
                publish_count = 0
                front_encode_s = wrist_encode_s = 0.0
                front_ros_publish_s = wrist_ros_publish_s = publish_work_s = 0.0
            rate.sleep()
    finally:
        stop_event.set()
        for worker in workers:
            worker.join(timeout=2.0)


if __name__ == "__main__":
    main()
