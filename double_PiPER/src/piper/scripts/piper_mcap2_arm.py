#!/usr/bin/env python3
"""Independent arm driver based on the single-arm leader/follower drivers."""
import json
import math
import os
import threading
import time

import rospy
from piper_sdk import Piper
from piper_sdk.piper_param.piper_param_manager import C_PiperParamManager
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

from piper_ctrl_follower_node import PiperFollowerNode, STATE_NAMES
from piper_ctrl_leader_node_new import PiperLeaderNode
from piper_command_validation import (
    clip_gripper, parse_follow_command, prepare_sdk_command, validate_pose,
)


class ArmDriver(PiperFollowerNode):
    # Reuse the single-arm pose file format and return-to-pose implementation.
    # Do not call the legacy constructor: it enables motors even when false.
    _wait_ctrl_mode = PiperLeaderNode._wait_ctrl_mode
    _wait_until_not_teaching = PiperLeaderNode._wait_until_not_teaching
    _return_to_teaching_mode = PiperLeaderNode._return_to_teaching_mode

    def __init__(self):
        rospy.init_node("piper_mcap2_arm")
        self.role = rospy.get_param("~role", "follower")
        if self.role not in ("leader", "follower"):
            raise ValueError("role must be leader or follower")
        self.can_port = rospy.get_param("~can_port")
        ports = rospy.get_param("/mcap2_can_ports", [])
        if ports and (len(ports) != 4 or len(set(ports)) != 4):
            raise ValueError("Four distinct CAN ports required")
        self.gripper_exist = self.have_gripper = True
        self.move_spd_rate_ctrl = int(rospy.get_param("~speed", 40))
        self.initial_pose_speed = int(rospy.get_param("~initial_pose_speed", 20))
        self.initial_pose_timeout = float(rospy.get_param("~initial_pose_timeout", 15.0))
        self.initial_pose_mode_timeout = float(rospy.get_param("~mode_timeout", 10.0))
        self.initial_pose_joint_tolerance = float(rospy.get_param("~initial_pose_joint_tolerance", .035))
        self.initial_pose_gripper_tolerance = float(rospy.get_param("~initial_pose_gripper_tolerance", .005))
        self.initial_pose_file = os.path.expanduser(rospy.get_param("~initial_pose_file", "~/.config/piper/mcap2.yaml"))
        self.feedback_timeout = float(rospy.get_param("~feedback_timeout", 1.0))
        self.monitor_feedback = bool(rospy.get_param("~monitor_feedback", False))
        self.auto_follow = bool(rospy.get_param("~auto_follow", True)) and not bool(
            rospy.get_param("~check_start_alignment", False))
        self.zero_angle_tolerance = float(rospy.get_param("~zero_angle_tolerance", .05))
        if not 0 <= self.zero_angle_tolerance <= .1:
            raise ValueError("zero_angle_tolerance must be in [0, 0.1] rad")
        if min(self.feedback_timeout, self.initial_pose_timeout, self.initial_pose_mode_timeout) <= 0:
            raise ValueError("Timeouts must be positive")
        if not all(1 <= value <= 100 for value in (self.move_spd_rate_ctrl, self.initial_pose_speed)):
            raise ValueError("Speeds must be in 1..100")
        if min(self.initial_pose_joint_tolerance, self.initial_pose_gripper_tolerance) < 0:
            raise ValueError("Tolerances must be nonnegative")
        self.return_leader_with_follower = False
        self.command_lock = threading.RLock()
        self.operation_lock = threading.Lock()
        self.enable_cancelled = threading.Event()
        self.leader_control_blocked = True
        self.initial_pose_return_active = False
        self.enabled = False
        self.mode_transition = False
        self.fault = None
        self.diagnostic = "waiting for feedback"
        self.ready = False
        self.last_stamps = [0., 0., 0.]
        self.last_updates = [time.monotonic()] * 3
        self.leader_stamp = 0.
        self.params = C_PiperParamManager()
        self.piper = Piper(self.can_port)
        self.interface = self.piper.init()
        self.piper.connect()
        leaf = "joint_grip_pose" if self.role == "leader" else "current_joint_obs"
        self.joint_obs_pub = rospy.Publisher(leaf, JointState, queue_size=10)
        self.applied_cmd_pub = rospy.Publisher("applied_joint_cmd", JointState, queue_size=10)
        self.status_pub = rospy.Publisher("status", String, queue_size=1, latch=True)
        for name in ("enable", "enter_teaching_mode", "start_following", "stop_following",
                     "save_initial_pose", "go_initial_pose", "reset_fault", "recover_stop", "emergency_stop"):
            rospy.Service(name, Trigger, self.service(name))
        if self.role == "follower":
            rospy.Subscriber("joint_ctrl_single", JointState, self.joint_callback, queue_size=1)
        rospy.on_shutdown(self.shutdown)
        self.timer = rospy.Timer(rospy.Duration(.02), self.tick)
        if rospy.get_param("~auto_enable", True):
            response = self.service("enable")(None)
            (rospy.loginfo if response.success else rospy.logerr)("%s: %s", self.can_port, response.message)
            if response.success and self.role == "follower" and self.auto_follow:
                with self.command_lock:
                    if not self.enable_cancelled.is_set() and not rospy.is_shutdown():
                        response = self.service("start_following")(None)
                        (rospy.loginfo if response.success else rospy.logerr)("%s startup: %s", self.can_port, response.message)

    def service(self, name):
        def invoke(request):
            # Emergency stop must remain available during a long home operation.
            if name == "emergency_stop":
                with self.command_lock:
                    self.enable_cancelled.set()
                    self.trip("Operator emergency stop")
                return TriggerResponse(True, "Emergency stop requested")
            if name == "stop_following":
                return self.stop_following(request)
            if not self.operation_lock.acquire(False):
                return TriggerResponse(False, "Another arm operation is active")
            try:
                return getattr(self, name)(request)
            except Exception as exc:
                return TriggerResponse(False, "%s: %s" % (self.can_port, exc))
            finally:
                self.operation_lock.release()
        return invoke

    def sample(self):
        joints, joint_stamp, _ = self.piper.get_joint_states()
        grip, grip_stamp, _ = self.piper.get_gripper_states()
        status = self.interface.GetArmStatus()
        stamps = [joint_stamp, grip_stamp, status.time_stamp]
        names = ("joints", "gripper", "arm_status")
        now = time.monotonic()
        for index, stamp in enumerate(stamps):
            if stamp <= 0:
                raise RuntimeError("%s feedback not received" % names[index])
            if self.monitor_feedback and stamp < self.last_stamps[index]:
                raise RuntimeError("feedback time moved backwards")
            if stamp > self.last_stamps[index]:
                self.last_updates[index] = now
            if self.monitor_feedback and now - self.last_updates[index] > self.feedback_timeout:
                raise RuntimeError("%s feedback stale" % names[index])
        self.last_stamps = stamps
        arm = status.arm_status
        teaching = self.role == "leader" and int(arm.ctrl_mode) == 2 and int(arm.arm_status) == 0x0B
        if int(arm.err_code) or (int(arm.arm_status) != 0 and not teaching):
            raise RuntimeError("arm_status=%s err_code=0x%04X ctrl_mode=%s" %
                               (arm.arm_status, int(arm.err_code), arm.ctrl_mode))
        if self.enabled and self.monitor_feedback:
            expected = (1, 2) if self.role == "leader" else (1,)
            bad_mode = not self.mode_transition and int(arm.ctrl_mode) not in expected
            disabled = self.role == "follower" and not all(self.interface.GetArmEnableStatus())
            if bad_mode or disabled:
                raise RuntimeError("motor disabled or unexpected control mode")
        values = tuple(float(v) for v in joints) + (float(grip[0]),)
        if len(values) != 7 or not all(math.isfinite(v) for v in values):
            raise RuntimeError("invalid seven-dimensional feedback")
        return values, min(stamps[:2])

    def tick(self, _):
        with self.command_lock:
            try:
                values, stamp = self.sample()
                self.ready, self.diagnostic = True, ""
                msg = JointState()
                msg.header.stamp = rospy.Time.from_sec(stamp)
                msg.name, msg.position = STATE_NAMES, values
                self.joint_obs_pub.publish(msg)
                if self.monitor_feedback and self.role == "follower" and not self.leader_control_blocked:
                    if rospy.Time.now().to_sec() - self.leader_stamp > self.feedback_timeout:
                        raise RuntimeError("leader commands stale")
            except Exception as exc:
                self.ready, self.diagnostic = False, str(exc)
                if self.enabled:
                    self.trip(str(exc))
            self.status_pub.publish(String(data=json.dumps({
                "can": self.can_port, "role": self.role, "ready": self.ready,
                "enabled": self.enabled, "fault": self.fault, "diagnostic": self.diagnostic,
                "following": self.role == "follower" and not self.leader_control_blocked,
                "monitor_feedback": self.monitor_feedback,
                "returning": self.initial_pose_return_active})))

    def enable(self, _):
        if self.fault:
            raise RuntimeError("Resolve fault, then reset_fault/recover_stop explicitly")
        # Each arm initializes independently in the same enable -> CAN mode
        # order as the legacy drivers. No four-arm feedback prerequisite.
        with self.command_lock:
            self.enable_cancelled.clear()
            self.leader_control_blocked = True
            self.enabled = False
        deadline = time.monotonic() + self.initial_pose_mode_timeout
        motors_ready = False
        detail = "arm status feedback not received"
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self.command_lock:
                if self.enable_cancelled.is_set():
                    raise RuntimeError("Initialization cancelled by stop request")
                arm = self.interface.GetArmStatus()
                if arm.time_stamp > 0:
                    if int(arm.arm_status.err_code) or int(arm.arm_status.arm_status) not in (0, 0x0B):
                        raise RuntimeError("%s; resolve stop/error before enabling" % arm)
                    if self.fault:
                        raise RuntimeError(self.fault)
                    if not motors_ready:
                        motors_ready = bool(self.piper.enable_arm())
                        if motors_ready:
                            self.piper.enable_gripper()
                    flags = list(self.interface.GetArmEnableStatus())
                    detail = "ctrl_mode=%s, enabled=%s" % (arm.arm_status.ctrl_mode, flags)
                    if motors_ready:
                        # Like the single-arm driver, retry mode switching until
                        # feedback confirms it, rather than sending only once.
                        if int(arm.arm_status.ctrl_mode) == 2:
                            self.interface.MotionCtrl_1(0, 0, 2)
                        if int(arm.arm_status.ctrl_mode) != 1:
                            self.interface.ModeCtrl(1, 1, self.move_spd_rate_ctrl, 0)
                        else:
                            try:
                                self.sample()
                                if all(flags):
                                    self.enabled = True
                                    return TriggerResponse(True, "Enabled in CAN control; following remains stopped")
                            except RuntimeError as exc:
                                detail += "; "+str(exc)
            time.sleep(.02)
        if rospy.is_shutdown() or self.enable_cancelled.is_set():
            raise RuntimeError("Initialization cancelled by shutdown/stop request")
        raise RuntimeError("enable/CAN mode timeout: "+detail)

    def enter_teaching_mode(self, _):
        if self.role != "leader" or not self.enabled or self.fault:
            raise RuntimeError("A healthy enabled leader is required")
        if int(self.interface.GetArmStatus().arm_status.ctrl_mode) == 2:
            self.sample()
            return TriggerResponse(True, "Leader already in teaching mode")
        self.mode_transition = True
        try:
            if not self._return_to_teaching_mode() or self.fault:
                self.trip("teaching mode transition failed")
                raise RuntimeError(self.fault)
        finally:
            self.mode_transition = False
        return TriggerResponse(True, "Leader in teaching mode")

    def start_following(self, _):
        with self.command_lock:
            if self.role != "follower" or not self.enabled or self.fault:
                raise RuntimeError("A healthy enabled follower is required")
            self.sample()
            # Set a brief grace period for the first command, not a fake action.
            self.leader_stamp = rospy.Time.now().to_sec()
            self.leader_control_blocked = False
        return TriggerResponse(True, "Following enabled")

    def stop_following(self, _):
        with self.command_lock:
            self.enable_cancelled.set()
            self.leader_control_blocked = True
        return TriggerResponse(True, "New leader targets blocked; motors keep holding")

    def joint_callback(self, msg):
        with self.command_lock:
            if self.leader_control_blocked or self.fault or not self.enabled:
                return
            try:
                stamp = msg.header.stamp.to_sec()
                if self.monitor_feedback and abs(rospy.Time.now().to_sec() - stamp) > self.feedback_timeout:
                    raise RuntimeError("stale leader sample")
                self.sample()
                values = parse_follow_command(msg.position)
                # Match the working single-arm/legacy follower's gripper range
                # handling. Keep raw leader/state feedback unchanged; publish
                # the bounded value actually passed to the SDK as Action.
                gripper = clip_gripper(values[6])
                if gripper != values[6]:
                    rospy.logwarn_throttle(2., "Gripper command %.3f mm limited to %.3f mm" %
                                           (values[6]*1000, gripper*1000))
                joints, grip = self._validate_initial_pose({"joint_positions": list(msg.position[:6]),
                                                           "gripper": gripper})
                self.leader_stamp = stamp
                self._dispatch_joint_command(joints, grip, self.move_spd_rate_ctrl)
            except Exception as exc:
                self.trip(str(exc))

    def _dispatch_joint_command(self, joints, gripper, speed):
        if self.fault or not self.enabled or not self.piper.get_ctrl_output_status():
            raise RuntimeError("Follower output is not enabled")
        self.sample()
        joints, gripper = prepare_sdk_command(joints, gripper, self._validate_initial_pose)
        self.piper.move_j(joints, speed)
        self.piper.move_gripper(gripper, 1)
        applied = JointState()
        applied.header.stamp = rospy.Time.now()
        applied.name, applied.position = STATE_NAMES, joints + [gripper]
        self.applied_cmd_pub.publish(applied)

    def _validate_initial_pose(self, value):
        return validate_pose(value, self.params.GetJointLimitParam,
                             self.zero_angle_tolerance, error_style="dual")

    def save_initial_pose(self, request):
        with self.command_lock:
            if self.role != "follower" or not self.leader_control_blocked:
                raise RuntimeError("Stop following before saving follower pose")
            self.sample()
            return super().save_initial_pose(request)

    def go_initial_pose(self, request):
        if self.role != "follower" or not self.enabled or self.fault:
            raise RuntimeError("Healthy enabled follower required for pose return")
        self.sample()
        return super().go_initial_pose(request)

    def trip(self, reason):
        with self.command_lock:
            first = not self.fault
            self.fault = self.fault or str(reason)
            self.leader_control_blocked = True
            self.enabled = False
            if first and self.role == "follower":
                self.interface.EmergencyStop(1)
            if first:
                rospy.logerr("%s fault: %s", self.can_port, reason)

    def reset_fault(self, _):
        with self.command_lock:
            self.sample()
            self.fault = None
            self.enabled = False
            self.leader_control_blocked = True
        return TriggerResponse(True, "Fault acknowledged; explicit enable required")

    def recover_stop(self, _):
        with self.command_lock:
            if self.enabled or self.initial_pose_return_active or self.role != "follower":
                raise RuntimeError("Recovery only allowed on inactive followers")
            status = self.interface.GetArmStatus()
            if status.time_stamp <= 0 or int(status.arm_status.err_code):
                raise RuntimeError("Missing feedback or hardware error; no recovery sent")
            if int(status.arm_status.arm_status) not in (0, 1):
                raise RuntimeError("Only error-free emergency stop may be recovered")
            if int(status.arm_status.arm_status) == 1:
                self.interface.MotionCtrl_1(2, 6, 2)
            self.leader_control_blocked = True
        deadline = time.monotonic() + self.initial_pose_mode_timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            try:
                with self.command_lock:
                    self.sample()
                    self.fault = None
                return TriggerResponse(True, "Stop recovered; enable explicitly before following")
            except RuntimeError:
                time.sleep(.02)
        raise RuntimeError("Stop recovery timed out")

    def shutdown(self):
        with self.command_lock:
            self.enable_cancelled.set()
            self.leader_control_blocked = True
            if self.role == "follower":
                if self.fault:
                    self.interface.EmergencyStop(1)
                elif self.enabled:
                    self.interface.MotionCtrl_1(0, 6, 0)


if __name__ == "__main__":
    ArmDriver()
    rospy.spin()
