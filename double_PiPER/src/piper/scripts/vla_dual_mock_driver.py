#!/usr/bin/env python3
"""Dual bringup test driver: ROS messages only, no SDK/CAN/device imports."""
import threading
import json
import time

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from dual_feedback import FeedbackTracker, FRAMES


class MockDriver:
    def __init__(self):
        rospy.init_node("vla_dual_mock_driver")
        self.lock = threading.Lock()
        self.position = [0.0] * 6 + [0.04]
        self.feedback = FeedbackTracker()
        self.feedback_timeout = float(rospy.get_param("~feedback_timeout_s", .5))
        if not 0 < self.feedback_timeout < float("inf"):
            raise ValueError("feedback_timeout_s must be finite and positive")
        self.feedback_enabled = True  # Mock only: False simulates stopped CAN.
        self.feedback_pub = rospy.Publisher("feedback_health", String, queue_size=1)
        self.publisher = rospy.Publisher("joint_states_single", JointState, queue_size=1)
        self.subscriber = rospy.Subscriber(
            "joint_ctrl_single", JointState, self.command, queue_size=1)

    def command(self, message):
        # The controller supplies the same seven physical values as a real
        # driver. This mock applies no extra conversion or safety limits.
        with self.lock:
            self.position = list(message.position)

    def publish_once(self):
        with self.lock:
            now = rospy.Time.now()
            ros_stamp = float(now.to_sec() if hasattr(now, 'to_sec') else now)
            if self.feedback_enabled:
                source_stamp = time.time()
                for frame_id in FRAMES.values():
                    self.feedback.received(frame_id, source_stamp, ros_stamp)
            self.feedback_pub.publish(json.dumps(self.feedback.report(self.feedback_timeout)))
            stamp = self.feedback.take_stamp(self.feedback_timeout)
            if stamp is None:
                return
            state = JointState()
            state.header.stamp = rospy.Time.from_sec(stamp) if hasattr(rospy.Time, 'from_sec') else stamp
            state.name = ["joint%d" % i for i in range(1, 7)] + ["gripper"]
            state.position = list(self.position)
        self.publisher.publish(state)

    def run(self):
        rate = rospy.Rate(200)
        while not rospy.is_shutdown():
            self.publish_once()
            rate.sleep()


if __name__ == "__main__":
    MockDriver().run()
