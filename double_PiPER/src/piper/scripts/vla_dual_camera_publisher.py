#!/usr/bin/env python3
"""Three independently captured JPEG streams for dual PiPER inference."""
import threading

import cv2
import rospy
from sensor_msgs.msg import CompressedImage

from dual_compressed_camera_publisher import (
    CameraCaptureWorker, LatestFrame, make_message, validate_camera_config,
    wait_for_workers,
)


class DualVlaCameras:
    def __init__(self):
        self.stop_event = threading.Event()
        self.workers = []
        self.views = {}
        self.publishers = {}
        self.publish_fps = float(rospy.get_param("~publish_fps", 20))
        self.quality = int(rospy.get_param("~jpeg_quality", 90))
        period = float(rospy.get_param("~diagnostic_period_s", 5.0))
        debug = bool(rospy.get_param("~camera_debug_stats", False))
        if self.publish_fps <= 0 or period <= 0 or not 1 <= self.quality <= 100:
            raise ValueError("Invalid camera publishing settings")
        devices = set()
        for label, default in (("front", "/dev/video4"),
                               ("left_wrist", "/dev/video2"),
                               ("right_wrist", "/dev/video0")):
            device = rospy.get_param("~" + label + "_camera", default)
            if device in devices:
                raise ValueError("Three distinct camera devices required")
            devices.add(device)
            width = int(rospy.get_param("~" + label + "_width", 640))
            height = int(rospy.get_param("~" + label + "_height", 480))
            fourcc = rospy.get_param("~" + label + "_fourcc", "MJPG")
            fps = float(rospy.get_param("~" + label + "_capture_fps", 30))
            validate_camera_config(label, fourcc, fps)
            if width <= 0 or height <= 0:
                raise ValueError("Camera dimensions must be positive")
            latest = LatestFrame()
            self.views[label] = latest
            self.publishers[label] = rospy.Publisher(
                "/camera/" + label + "/compressed", CompressedImage, queue_size=1)
            self.workers.append(CameraCaptureWorker(
                label, device, cv2.CAP_V4L2, width, height, fps, fourcc,
                latest, self.stop_event, debug, period))

    def publish_once(self):
        for label, view in self.views.items():
            frame, stamp, _captured = view.get_copy()
            if frame is None:
                rospy.logwarn_throttle(1.0, "No %s camera frame available", label)
                continue
            message, _encode_s = make_message(
                frame, stamp, label + "_camera", self.quality)
            self.publishers[label].publish(message)

    def close(self):
        self.stop_event.set()
        for worker in self.workers:
            if worker.ident is not None:
                worker.join(timeout=2.0)

    def run(self):
        try:
            for worker in self.workers:
                worker.start()
            wait_for_workers(self.workers)
            rate = rospy.Rate(self.publish_fps)
            while not rospy.is_shutdown():
                for worker in self.workers:
                    if worker.error is not None:
                        raise worker.error
                self.publish_once()
                rate.sleep()
        finally:
            self.close()


if __name__ == "__main__":
    rospy.init_node("vla_dual_camera_publisher")
    DualVlaCameras().run()
