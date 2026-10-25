#!/usr/bin/env python3
"""Dual follower driver: unchanged command path, receive-stamped JointState."""
import json
import math
import threading
import time

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from piper_sdk import C_PiperInterface

from dual_feedback import FeedbackTracker, FRAMES
from piper_ctrl_single_node import C_PiperRosNode


class TrackedPiperInterface(C_PiperInterface):
    # Keep this subclass out of the base SDK singleton cache.
    _instances = {}

    def __init__(self, *args, **kwargs):
        if not hasattr(self, 'feedback_tracker'):
            self.feedback_lock = threading.RLock()
            self.feedback_tracker = FeedbackTracker()
        super().__init__(*args, **kwargs)

    def ParseCANFrame(self, frame):
        with self.feedback_lock:
            # Only a successful SDK decode of a real, complete feedback frame
            # may advance provenance. No getters or ROS timers increment it.
            if frame is None:
                return
            tracked = frame.arbitration_id in FRAMES.values()
            if tracked:
                key = next(key for key, value in FRAMES.items() if value == frame.arbitration_id)
                old = self.feedback_tracker.frames.get(key)
                stamp = float(frame.timestamp)
                if (len(frame.data) != 8 or frame.is_error_frame or frame.is_remote_frame
                        or frame.is_extended_id or not math.isfinite(stamp) or stamp <= 0
                        or stamp > time.time()
                        or (old is not None and stamp <= old['source_stamp'])):
                    return
            super().ParseCANFrame(frame)
            if tracked:
                if frame.arbitration_id == FRAMES['status']:
                    status = self.GetArmStatus().arm_status
                    error = status.err_status
                    self.feedback_tracker.communication_error = status.arm_status == 0x05 or any(
                        getattr(error, 'communication_status_joint_%d' % j) for j in range(1, 7))
                self.feedback_tracker.received(frame.arbitration_id, float(frame.timestamp),
                                               rospy.Time.now().to_sec())


class DualFeedbackNode(C_PiperRosNode):
    def _create_piper_interface(self):
        self.start_sdk_joint_limit = rospy.get_param('~start_sdk_joint_limit', True)
        if type(self.start_sdk_joint_limit) is not bool:
            raise ValueError('~start_sdk_joint_limit must be a ROS boolean')
        return TrackedPiperInterface(can_name=self.can_port,
                                     start_sdk_joint_limit=self.start_sdk_joint_limit)

    def __init__(self):
        super().__init__()
        self.feedback_timeout = float(rospy.get_param('~feedback_timeout_s', 0.5))
        if not 0 < self.feedback_timeout < float('inf'):
            raise ValueError('feedback_timeout_s must be finite and positive')
        self.feedback_pub = rospy.Publisher('feedback_health', String, queue_size=1)

    def PublishArmJointAndGripper(self):
        with self.piper.feedback_lock:
            tracker = self.piper.feedback_tracker
            health = tracker.report(self.feedback_timeout)
            # SDK isOk / FPS are diagnostic, NOT proofs of frame reception.
            health['sdk_is_ok'] = self.piper.isOk()
            health['sdk_can_fps'] = self.piper.GetCanFps()
            self.feedback_pub.publish(json.dumps(health))
            stamp = tracker.take_stamp(self.feedback_timeout)
            if stamp is None:
                return  # Never republish a cache entry with a new source stamp.
            joint = self.piper.GetArmJointMsgs().joint_state
            grip = self.piper.GetArmGripperMsgs().gripper_state
            high = self.piper.GetArmHighSpdInfoMsgs()
            state = JointState()
            state.header.stamp = rospy.Time.from_sec(stamp)
            state.name = ['joint%d' % j for j in range(1, 7)] + ['gripper']
            # Preserve existing driver conversions, rad/m layout and topic.
            state.position = [(getattr(joint, 'joint_%d' % j)/1000)*0.017444
                              for j in range(1, 7)] + [grip.grippers_angle/1000000]
            state.velocity = [getattr(high, 'motor_%d' % j).motor_speed/1000
                              for j in range(1, 7)] + [0.]
            state.effort = [getattr(high, 'motor_%d' % j).effort/1000
                            for j in range(1, 7)] + [grip.grippers_effort/1000]
            self.joint_pub.publish(state)


if __name__ == '__main__':
    DualFeedbackNode().Pubilsh()
