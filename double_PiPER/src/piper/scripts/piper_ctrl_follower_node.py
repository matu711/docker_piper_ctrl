#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Leader-following PiPER driver with measured EE state and home services."""

import math
import os
import threading
import time

import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from piper_msgs.msg import PiperEulerPose
from piper_sdk import Piper
from piper_sdk.piper_param.piper_param_manager import C_PiperParamManager
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from tf.transformations import quaternion_from_euler

from piper_command_validation import (
    clip_gripper, parse_follow_command, prepare_sdk_command, validate_pose,
)


JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
STATE_NAMES = JOINT_NAMES + ["gripper"]


class PiperFollowerNode:
    def __init__(self):
        rospy.init_node("piper_ctrl_follower_node")
        self.can_port = rospy.get_param("~can_port", "can0")
        self.auto_enable = rospy.get_param("~auto_enable", True)
        self.gripper_exist = rospy.get_param("~gripper_exist", True)
        self.move_spd_rate_ctrl = 40
        self.initial_pose_speed = int(rospy.get_param("~initial_pose_speed", 20))
        self.initial_pose_timeout = float(rospy.get_param("~initial_pose_timeout", 15.0))
        self.initial_pose_joint_tolerance = float(rospy.get_param("~initial_pose_joint_tolerance", 0.035))
        self.initial_pose_gripper_tolerance = float(rospy.get_param("~initial_pose_gripper_tolerance", 0.005))
        self.initial_pose_file = os.path.expanduser(rospy.get_param("~initial_pose_file", "~/.config/piper/initial_pose.yaml"))
        self.ee_frame_id = rospy.get_param("~ee_frame_id", "piper_base")
        self.return_leader_with_follower = rospy.get_param("~return_leader_with_follower", False)
        self.leader_initial_pose_service = rospy.get_param("~leader_initial_pose_service", "/leader/go_initial_pose")
        self.have_gripper = True
        self.command_lock = threading.RLock()
        self.command_fault = None
        self.leader_control_blocked = False
        self.initial_pose_return_active = False
        self.latest_pose = None
        self.zero_angle_tolerance = float(rospy.get_param("~zero_angle_tolerance", 0.05))
        if not 0 <= self.zero_angle_tolerance <= 0.1:
            raise ValueError("zero_angle_tolerance must be in [0, 0.1] rad")
        self.params = C_PiperParamManager()
        if not 1 <= self.initial_pose_speed <= 100:
            raise ValueError("~initial_pose_speed must be in 1..100")
        if self.initial_pose_timeout <= 0.0 or self.initial_pose_joint_tolerance < 0.0 or self.initial_pose_gripper_tolerance < 0.0:
            raise ValueError("initial-pose timeout must be positive and tolerances non-negative")

        self.piper = Piper(self.can_port)
        self.interface = self.piper.init()
        self.piper.connect()
        time.sleep(0.2)
        while not self.piper.enable_arm():
            time.sleep(0.01)
        self.piper.enable_gripper()
        if self.interface.GetArmStatus().arm_status.ctrl_mode != 1:
            self.stop()
        over_time = time.time() + 10.0
        while self.interface.GetArmStatus().arm_status.ctrl_mode != 1:
            if over_time < time.time():
                raise SystemExit("Failed to switch to CAN mode; exit teaching mode first")
            self.interface.ModeCtrl(0x01, 0x01, self.move_spd_rate_ctrl, 0x00)
            time.sleep(0.01)
        self.enable()
        if self.auto_enable:
            while not self.piper.enable_arm():
                time.sleep(0.01)
            if self.gripper_exist:
                self.piper.enable_gripper()
        self.interface.ModeCtrl(0x01, 0x01, self.move_spd_rate_ctrl, 0x00)

        rospy.Subscriber("joint_ctrl_single", JointState, self.joint_callback, queue_size=1)
        rospy.Subscriber("/end_pose_euler", JointState, self.pose_callback, queue_size=1)
        self.joint_obs_pub = rospy.Publisher("current_joint_obs", JointState, queue_size=10)
        self.applied_cmd_pub = rospy.Publisher("applied_joint_cmd", JointState, queue_size=10)
        self.end_pose_pub = rospy.Publisher("end_pose", PoseStamped, queue_size=10)
        self.end_pose_euler_pub = rospy.Publisher("end_pose_euler", PiperEulerPose, queue_size=10)
        rospy.Service("save_initial_pose", Trigger, self.save_initial_pose)
        rospy.Service("go_initial_pose", Trigger, self.go_initial_pose)
        rospy.Service("set_leader_control_block", SetBool, self.set_leader_control_block)
        rospy.Service("resume_leader_control", Trigger, self.resume_leader_control)
        rospy.Timer(rospy.Duration(0.02), self.publish_follower_state)
        rospy.loginfo("Follower %s ready. Initial pose file: %s", self.can_port, self.initial_pose_file)

    def _read_joint_and_gripper(self):
        joints = tuple(float(value) for value in self.piper.get_joint_states()[0])
        gripper = float(self.piper.get_gripper_states()[0][0])
        if len(joints) != 6:
            raise RuntimeError("SDK returned {} joints, expected 6".format(len(joints)))
        return joints, gripper

    def _read_end_pose_euler(self):
        # Piper SDK reads GetArmEndPoseMsgs() feedback: XYZ [m], RPY [rad].
        pose = self.piper.get_end_pose_euler()[0]
        if len(pose) != 6 or not all(math.isfinite(float(v)) for v in pose):
            raise RuntimeError("Invalid SDK end-pose feedback: {}".format(pose))
        return tuple(float(value) for value in pose)

    def publish_follower_state(self, _event):
        try:
            joints, gripper = self._read_joint_and_gripper()
            x, y, z, roll, pitch, yaw = self._read_end_pose_euler()
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Follower feedback unavailable: %s", exc)
            return
        stamp = rospy.Time.now()
        state = JointState()
        state.header.stamp, state.name, state.position = stamp, STATE_NAMES, list(joints) + [gripper]
        self.joint_obs_pub.publish(state)
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp, self.ee_frame_id
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation.x, pose.pose.orientation.y = qx, qy
        pose.pose.orientation.z, pose.pose.orientation.w = qz, qw
        self.end_pose_pub.publish(pose)
        euler = PiperEulerPose()
        euler.header.stamp, euler.header.frame_id = stamp, self.ee_frame_id
        euler.x, euler.y, euler.z = x, y, z
        euler.roll, euler.pitch, euler.yaw = roll, pitch, yaw
        self.end_pose_euler_pub.publish(euler)

    def pose_callback(self, msg):
        self.latest_pose = msg

    def joint_callback(self, msg):
        with self.command_lock:
            if self.leader_control_blocked or self.command_fault:
                # HOLD during initial-pose alignment is an expected state.
                # Do not spam the operator console for every leader sample.
                return
            try:
                values = parse_follow_command(msg.position)
                gripper = clip_gripper(values[6])
                self._dispatch_joint_command(values[:6], gripper, self.move_spd_rate_ctrl)
            except Exception as exc:
                self._trip_command_fault(str(exc))

    def _trip_command_fault(self, reason):
        with self.command_lock:
            first = self.command_fault is None
            self.command_fault = self.command_fault or str(reason) or "Unknown command failure"
            self.leader_control_blocked = True
            if first:
                # Use the SDK interface directly: stop() waits for a particular
                # arm pose and must not block a command-fault callback.
                self.interface.EmergencyStop(1)
                rospy.logerr("%s command fault: %s", self.can_port, reason)

    def _dispatch_joint_command(self, joints, gripper, speed):
        if self.command_fault:
            raise RuntimeError("Command fault is latched: %s" % self.command_fault)
        joints, gripper = prepare_sdk_command(joints, gripper, self._validate_initial_pose)
        stamp = rospy.Time.now()
        self.piper.move_j(joints, speed)
        if self.gripper_exist:
            self.piper.move_gripper(gripper, 1)
        applied = JointState()
        applied.header.stamp, applied.name, applied.position = stamp, STATE_NAMES, list(joints) + [gripper]
        self.applied_cmd_pub.publish(applied)

    def _validate_initial_pose(self, value):
        return validate_pose(value, self.params.GetJointLimitParam,
                             self.zero_angle_tolerance, error_style="single")

    def save_initial_pose(self, _request):
        try:
            joints, gripper = self._read_joint_and_gripper()
            ee_pose = self._read_end_pose_euler()
            payload = {"format": "piper_initial_pose_v1", "saved_at_unix": time.time(), "can_port": self.can_port,
                       "joint_positions": list(joints), "gripper": gripper,
                       "ee_pose_euler": dict(zip(["x", "y", "z", "roll", "pitch", "yaw"], ee_pose)),
                       "units": {"joints": "rad", "gripper": "m", "ee_xyz": "m", "ee_rpy": "rad"}}
            directory = os.path.dirname(self.initial_pose_file)
            if directory:
                os.makedirs(directory, exist_ok=True)
            with open(self.initial_pose_file, "w", encoding="utf-8") as stream:
                yaml.safe_dump(payload, stream, default_flow_style=False, sort_keys=False)
            rospy.loginfo("Saved initial pose: path=%s joints=%s gripper=%.6f m", self.initial_pose_file, list(joints), gripper)
            return TriggerResponse(True, "Saved initial pose to {}".format(self.initial_pose_file))
        except Exception as exc:
            rospy.logerr("Could not save initial pose: %s", exc)
            return TriggerResponse(False, "Could not save initial pose: {}".format(exc))

    def go_initial_pose(self, _request):
        # MCAP2 overrides fault handling; its constructor does not set this
        # single-arm latch. Keep the inherited MCAP2 return path unchanged.
        if getattr(self, "command_fault", None):
            return TriggerResponse(False, "Command fault is latched; resolve the hardware stop and restart the follower")
        try:
            with open(self.initial_pose_file, "r", encoding="utf-8") as stream:
                target_joints, target_gripper = self._validate_initial_pose(yaml.safe_load(stream))
        except Exception as exc:
            return TriggerResponse(False, "Could not load initial pose: {}".format(exc))
        with self.command_lock:
            self.leader_control_blocked = True
            self.initial_pose_return_active = True
        try:
            if self.return_leader_with_follower:
                try:
                    rospy.wait_for_service(self.leader_initial_pose_service, timeout=2.0)
                    leader_result = rospy.ServiceProxy(self.leader_initial_pose_service, Trigger)()
                except Exception as exc:
                    return TriggerResponse(False, "Leader initial-pose service failed (follower remains blocked): {}".format(exc))
                if not leader_result.success:
                    return TriggerResponse(False, "Leader did not reach initial pose (follower remains blocked): {}".format(leader_result.message))
            with self.command_lock:
                try:
                    self._dispatch_joint_command(target_joints, target_gripper, self.initial_pose_speed)
                except Exception as exc:
                    if not hasattr(self, "command_fault"):
                        raise  # MCAP2 retains its existing fault handling.
                    self._trip_command_fault(str(exc))
                    return TriggerResponse(False, "Initial-pose command failed; follower remains blocked: %s" % exc)
            deadline = time.monotonic() + self.initial_pose_timeout
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                if getattr(self, "command_fault", None):
                    return TriggerResponse(False, "Command fault during initial-pose return; follower remains blocked")
                try:
                    joints, gripper = self._read_joint_and_gripper()
                    joint_error = max(abs(now - target) for now, target in zip(joints, target_joints))
                    if joint_error <= self.initial_pose_joint_tolerance and abs(gripper - target_gripper) <= self.initial_pose_gripper_tolerance:
                        message = "Leader and follower reached initial pose" if self.return_leader_with_follower else "Follower reached initial pose"
                        return TriggerResponse(True, message + "; leader control remains blocked until resumed")
                except Exception as exc:
                    rospy.logwarn_throttle(1.0, "Waiting for initial pose feedback: %s", exc)
                time.sleep(0.05)
            return TriggerResponse(False, "Timed out returning to initial pose; leader control remains blocked")
        finally:
            with self.command_lock:
                self.initial_pose_return_active = False

    def set_leader_control_block(self, request):
        with self.command_lock:
            if self.command_fault and not request.data:
                return SetBoolResponse(False, "Command fault is latched; follower remains blocked")
            if self.initial_pose_return_active and not request.data:
                return SetBoolResponse(False, "Initial-pose return is still active")
            self.leader_control_blocked = bool(request.data)
        return SetBoolResponse(True, "Leader control {}".format("blocked" if request.data else "resumed"))

    def resume_leader_control(self, _request):
        with self.command_lock:
            if self.command_fault:
                return TriggerResponse(False, "Command fault is latched; resolve the hardware stop and restart the follower")
            if self.initial_pose_return_active:
                return TriggerResponse(False, "Initial-pose return is still active")
            self.leader_control_blocked = False
        return TriggerResponse(True, "Leader control resumed")

    def get_pos(self):
        joints, gripper = self._read_joint_and_gripper()
        return joints + (gripper,) if self.have_gripper else joints

    def stop(self):
        self.interface.EmergencyStop(0x01)
        time.sleep(1.0)
        limit_angle, pos = [0.1745, 0.7854, 0.2094], self.get_pos()
        while not (abs(pos[1]) < limit_angle[0] and abs(pos[2]) < limit_angle[0] and limit_angle[2] < pos[4] < limit_angle[1]):
            time.sleep(0.01)
            pos = self.get_pos()
        self.piper.disable_arm()
        time.sleep(1.0)

    def enable(self):
        while not self.piper.enable_arm():
            time.sleep(0.01)
        if self.have_gripper:
            time.sleep(0.01)
            self.piper.enable_gripper()


if __name__ == "__main__":
    PiperFollowerNode()
    rospy.spin()
