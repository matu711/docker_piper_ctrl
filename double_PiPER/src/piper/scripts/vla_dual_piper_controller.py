#!/usr/bin/env python3
"""Left-then-right absolute joint actions; validate both before dispatch.

The unchanged single-arm controller supplies every per-arm safety operation.
Only its final dispatch is staged, so a rejected right arm cannot move the left.
Joint steps are clipped against fresh measured follower state, at Runtime's
20 Hz execution rate. No CAN interface or policy delta reconstruction lives here.
"""
import math
import json
import threading
import time
import uuid

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray, String
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse
from piper_sdk import C_PiperForwardKinematics
from piper_sdk.piper_param.piper_param_manager import C_PiperParamManager

from vla_piper_controller import VlaPiperController
from dual_feedback import FeedbackWatch


class StagedArm(VlaPiperController):
    """Reuse joint_absolute safety without initializing another ROS node."""

    def __init__(self, side, namespace):
        self.action_type = "joint_absolute"
        self.gripper_action_type = "physical_absolute"
        # Defaults match the Python controller. The launch passes the same
        # launch-specific defaults as start_vla_piper.launch (joint margin 0.1).
        self.z_min = float(rospy.get_param("~z_min", 0.01))
        self.z_enabled = bool(rospy.get_param("~z_enabled", True))
        self.dry_run = bool(rospy.get_param("~dry_run", True))
        self.dh_is_offset = int(rospy.get_param("~dh_is_offset", 1))
        self.joint_limit_enabled = rospy.get_param("~controller_joint_limit_enabled", True)
        if type(self.joint_limit_enabled) is not bool:
            raise ValueError("controller_joint_limit_enabled must be a ROS boolean")
        rospy.loginfo("Dual Controller side=%s controller_joint_limit_enabled=%s",
                      side, self.joint_limit_enabled)
        for name, default in (
            ("gripper_closed_width_m", 0.0), ("gripper_open_width_m", 0.10),
            ("gripper_saturation_margin_m", 0.01),
            ("joint_saturation_margin_rad", 0.05),
        ):
            setattr(self, name, float(rospy.get_param("~" + name, default)))
        self._validate_gripper_config()
        self._validate_joint_saturation_config()
        self.fk = C_PiperForwardKinematics(dh_is_offset=self.dh_is_offset)
        self.params = C_PiperParamManager()
        self._reset_pbl8_reference()
        self.prepared = None
        self.joint_command_pub = rospy.Publisher(
            namespace + "/joint_states", JointState, queue_size=1)
        self.target_joint_pub = rospy.Publisher(
            "/vla_dual/" + side + "/target_joint_command", JointState, queue_size=10)
        self.target_z_pub = rospy.Publisher(
            "/safety/" + side + "/target_z", Float64, queue_size=10)
        self.safety_state_pub = rospy.Publisher(
            "/safety/" + side + "/state", String, queue_size=10)
        self.safety_reason_pub = rospy.Publisher(
            "/safety/" + side + "/reason", String, queue_size=10)

    def _apply_joint_saturation(self, target):
        if self.joint_limit_enabled:
            return super()._apply_joint_saturation(target)
        # Disabling SDK-derived ranges must never bypass shape/finite checks.
        self._require_action_size(target)
        self._require_finite_action(target)
        return list(target)

    def _publish_or_dry_run(self, publisher, command, source):
        self.prepared = (publisher, command, source)

    def _official_target_joint_z(self, target):
        z = super()._official_target_joint_z(target)
        if not math.isfinite(z):
            raise ValueError("FK target Z is not finite")
        return z

    def commit(self):
        # The base implementation retains precisely the single-arm dry-run
        # and successful-send diagnostics. It performs no further validation.
        super()._publish_or_dry_run(*self.prepared)


class VlaDualPiperController:
    def __init__(self):
        rospy.init_node("vla_dual_piper_controller")
        self.lock = threading.RLock()
        self.hold_requested = True
        self.fault_latched = False
        self.session_token = None
        self.initial_pose_active = False
        self.action_type = rospy.get_param("~action_type", "joint_absolute")
        VlaPiperController._validate_action_type(self.action_type)
        if self.action_type != "joint_absolute":
            raise ValueError("Dual controller requires joint_absolute")
        self.states = {}
        self.state_received = {}
        self.feedback = FeedbackWatch()
        # No guessed velocity/step default. Commissioned per-joint limits must
        # be supplied explicitly, in rad per 20 Hz action (not rad/s).
        self.max_delta = {}
        for side in ("left", "right"):
            name = "~max_delta_" + side
            values = rospy.get_param(name, None)
            if not isinstance(values, (list, tuple)) or len(values) != 6:
                raise ValueError(name + " requires six explicit rad/step limits at 20 Hz")
            if any(isinstance(v, bool) for v in values):
                raise ValueError(name + " must contain finite positive numbers")
            values = [float(v) for v in values]
            if any(not math.isfinite(v) or v <= 0 for v in values):
                raise ValueError(name + " must contain finite positive numbers")
            self.max_delta[side] = values
        # Match the current dual Runtime observation timeout; independently
        # check source stamps AND local monotonic reception age.
        self.state_timeout_s = float(rospy.get_param("~state_timeout_s", 0.5))
        if not math.isfinite(self.state_timeout_s) or self.state_timeout_s <= 0:
            raise ValueError("state_timeout_s must be finite and positive")
        left_ns = rospy.get_param("~left_namespace", "/followerL").rstrip("/")
        right_ns = rospy.get_param("~right_namespace", "/followerR").rstrip("/")
        if left_ns == right_ns or not left_ns or not right_ns:
            raise ValueError("Distinct nonempty follower namespaces required")
        self.arms = {"left": StagedArm("left", left_ns),
                     "right": StagedArm("right", right_ns)}
        self.state_pub = rospy.Publisher(
            "/vla/dual_joint_states", JointState, queue_size=1)
        self.safety_state_pub = rospy.Publisher("/safety/state", String, queue_size=10)
        self.safety_reason_pub = rospy.Publisher("/safety/reason", String, queue_size=10)
        # Non-latched: a completed fault must not stop a later Runtime session.
        self.fault_pub = rospy.Publisher("/vla_dual/controller_fault", String, queue_size=10)
        # One non-latched status stream, independent of Action reception. The
        # token is the existing session token, never an additional epoch/lease.
        self.status_rate_hz = rospy.get_param("~controller_status_rate_hz", 20.0)
        if (isinstance(self.status_rate_hz, bool)
                or not math.isfinite(self.status_rate_hz) or self.status_rate_hz <= 0):
            raise ValueError("controller_status_rate_hz must be finite and positive")
        self.status_pub = rospy.Publisher("/vla_dual/controller_status", String, queue_size=1)
        self.subscribers = [
            rospy.Subscriber("/vla_dual/approved_action", Float64MultiArray,
                             self.action_callback, queue_size=1),
            rospy.Subscriber("/vla_dual/hold", Bool, self.hold_callback, queue_size=1),
        ]
        for side, namespace in (("left", left_ns), ("right", right_ns)):
            self.subscribers.append(rospy.Subscriber(
                namespace + "/joint_states_single", JointState,
                self.state_callback, callback_args=side, queue_size=1))
            self.subscribers.append(rospy.Subscriber(
                namespace + "/feedback_health", String,
                self.feedback_callback, callback_args=side, queue_size=1))
        self.stop_service = rospy.Service("~stop", Trigger, self.stop)
        self.start_service = rospy.Service("/vla_dual/controller_start", Trigger, self.start)
        self.initial_pose_service = rospy.Service(
            "/vla_dual/initial_pose_lock", SetBool, self.initial_pose_lock)
        self.state_watchdog = rospy.Timer(rospy.Duration(1. / 20.), self.check_state_freshness)
        self.publish_status(None)
        self.status_timer = rospy.Timer(rospy.Duration(1. / self.status_rate_hz), self.publish_status)
        rospy.loginfo("Dual PiPER: absolute [left7, right7]; initial HOLD; dry_run=%s",
                      self.arms["left"].dry_run)

    def publish_status(self, _event):
        """Snapshot and publish under the same lock as start/stop/actions.

        stamp_ns is ROS time at snapshot, not callback receipt time. Receivers
        must check source age and advancing stamps as well as local silence.
        """
        with self.lock:
            state = ("FAULT" if self.fault_latched else
                     "HOLD" if self.hold_requested or self.session_token is None else "RUNNING")
            self.status_pub.publish(json.dumps(dict(
                schema=1, state=state, token=self.session_token or "",
                stamp_ns=rospy.Time.now().to_nsec())))

    def hold(self, reason):
        for arm in self.arms.values():
            arm.prepared = None
            arm._hold(reason)
        self.safety_state_pub.publish("HOLD")
        self.safety_reason_pub.publish(reason)

    def hold_callback(self, message):
        with self.lock:
            if message.data:
                self.hold_requested = True
                self.hold("hold_requested")
            elif not self.fault_latched and self.session_token is not None:
                self.hold_requested = False

    def start(self, _request):
        """Explicit Runtime start; response acknowledges processed HOLD release.

        A fresh token also rejects queued Actions from every earlier session.
        Repeated Bool(false) messages can never clear a fault latch.
        """
        with self.lock:
            if getattr(self, "initial_pose_active", False):
                return TriggerResponse(success=False, message="Initial-pose return owns both arms")
            try:
                self.current_joint_states()
            except (RuntimeError, ValueError, TypeError) as error:
                self.controller_fault(str(error))
                return TriggerResponse(success=False, message=str(error))
            for arm in self.arms.values():
                arm.prepared = None
            self.session_token = uuid.uuid4().hex
            self.fault_latched = False
            self.hold_requested = False
            return TriggerResponse(success=True, message=self.session_token)

    def initial_pose_lock(self, request):
        """Exclude VLA starts while the independent mover owns driver commands."""
        with self.lock:
            if request.data and self.initial_pose_active:
                return SetBoolResponse(success=False, message="Initial-pose return already active")
            self.initial_pose_active = bool(request.data)
            self.controller_fault("initial_pose_return")
            return SetBoolResponse(success=True, message="Initial-pose lock=%s; HOLD maintained" % request.data)

    def controller_fault(self, reason):
        """Latch both arms and ask Runtime to invalidate its active chunk."""
        first = not self.fault_latched
        self.fault_latched = True
        self.hold_requested = True
        self.hold(reason)
        if first:
            self.fault_pub.publish(reason)

    def stop(self, _request):
        with self.lock:
            self.controller_fault("controller_stop")
        return TriggerResponse(success=True, message="Both arms HOLD; no further commands")

    def action_callback(self, message):
        with self.lock:
            if self.hold_requested or self.fault_latched:
                self.hold("hold_requested")
                return
            dimensions = getattr(getattr(message, "layout", None), "dim", [])
            if (len(dimensions) != 1 or self.session_token is None
                    or dimensions[0].label != "controller_session:" + self.session_token):
                # Delayed/untagged packets must not move or fault a new session.
                rospy.logwarn_throttle(1.0, "Ignoring Action from an unacknowledged controller session")
                return
            for arm in self.arms.values():
                arm.prepared = None
            try:
                if len(message.data) != 14:
                    raise ValueError("Expected exactly 14 values [left7, right7], got %d" %
                                     len(message.data))
                # Validate ALL 14 values before either arm reaches FK/staging.
                actions = {"left": list(message.data[:7]), "right": list(message.data[7:])}
                for side, action in actions.items():
                    arm = self.arms[side]
                    arm._require_action_size(action)
                    arm._require_finite_action(action)
                # Reject large excursions BEFORE step clipping could hide them.
                bounded = {s: self.arms[s]._apply_joint_saturation(a) for s, a in actions.items()}
                current = self.current_joint_states()
                for side, target in bounded.items():
                    applied = list(target)
                    for j, maximum in enumerate(self.max_delta[side]):
                        delta = target[j] - current[side][j]
                        applied[j] = current[side][j] + min(maximum, max(-maximum, delta))
                    rospy.loginfo("JOINT_CLIP side=%s current=%s raw=%s range_clipped=%s "
                                  "step_clipped=%s max_delta=%s",
                                  side, current[side][:6], actions[side][:6], target[:6],
                                  applied[:6], self.max_delta[side])
                    # Reuse the original range -> FK -> Z -> gripper -> stage
                    # pipeline, now with the step-clipped physical target.
                    self.arms[side].handle_joint_absolute(applied)
                self.current_joint_states()  # FK/logging must not outlive the State validity window.
            except (RuntimeError, ValueError, TypeError, OverflowError) as error:
                self.controller_fault(str(error))
                return
            try:
                for side, arm in self.arms.items():
                    arm.commit()
                    arm.prepared = None
            except Exception as error:
                # ROS transports are sequential: an already-published command
                # cannot be rolled back. Suppress remaining sends and notify.
                self.controller_fault("dispatch_failed:%s:%s" % (side, error))
                return
            self.safety_state_pub.publish(
                "DRY_RUN" if self.arms["left"].dry_run else "OK")
            self.safety_reason_pub.publish("")

    @staticmethod
    def stamp_seconds(stamp):
        return float(stamp.to_sec() if hasattr(stamp, "to_sec") else stamp)

    def current_joint_states(self):
        now, ros_now = time.monotonic(), self.stamp_seconds(rospy.Time.now())
        if not math.isfinite(ros_now) or ros_now <= 0:
            raise RuntimeError("invalid ROS time for State freshness")
        result = {}
        for side, arm in self.arms.items():
            self.feedback.check(side, self.state_timeout_s)
            if side not in self.states or side not in self.state_received:
                raise RuntimeError(side + ": missing Follower State")
            message = self.states[side]
            age = ros_now - self.stamp_seconds(message.header.stamp)
            local_age = now - self.state_received[side]
            if not 0 <= age <= self.state_timeout_s or not 0 <= local_age <= self.state_timeout_s:
                raise RuntimeError(side + ": stale/future Follower State")
            values = list(message.position)
            # Measured-state range checks follow the same switch as targets;
            # freshness, provenance and finite-value checks remain mandatory.
            if arm.joint_limit_enabled:
                for j, value in enumerate(values[:6], 1):
                    lower, upper = arm.params.GetJointLimitParam("j%d" % j)
                    if not lower <= value <= upper:
                        raise RuntimeError(side + ": measured joint%d outside SDK limits" % j)
            result[side] = values
        return result

    def check_state_freshness(self, _event):
        with self.lock:
            if self.hold_requested or self.fault_latched:
                return
            try:
                self.current_joint_states()
            except (RuntimeError, ValueError, TypeError) as error:
                self.controller_fault(str(error))

    def feedback_callback(self, message, side):
        with self.lock:
            try:
                self.feedback.update(side, json.loads(message.data), self.state_timeout_s)
            except (ValueError, TypeError, KeyError, AttributeError, OverflowError) as error:
                # Keep valid restart metadata, but invalidate the joint sample.
                # No later heartbeat or State clears the operational latch.
                if side in self.feedback.arms:
                    self.feedback.arms[side]["healthy"] = False
                self.states.pop(side, None)
                self.state_received.pop(side, None)
                self.controller_fault(side + ": CAN feedback: " + str(error))

    def state_callback(self, message, side):
        with self.lock:
            try:
                names = ["joint%d" % j for j in range(1, 7)] + ["gripper"]
                if (len(message.position) != 7 or len(message.name) != 7
                        or set(message.name) != set(names)):
                    raise ValueError("State requires joint1..6 and gripper exactly once")
                values = [float(message.position[message.name.index(n)]) for n in names]
                stamp = self.stamp_seconds(message.header.stamp)
                if not all(math.isfinite(v) for v in values) or not math.isfinite(stamp) or stamp <= 0:
                    raise ValueError("State contains invalid values/timestamp")
                if side in self.states and stamp < self.stamp_seconds(self.states[side].header.stamp):
                    raise ValueError("State timestamp moved backwards")
            except (ValueError, TypeError, OverflowError) as error:
                self.states.pop(side, None)
                self.state_received.pop(side, None)
                self.controller_fault(side + ": invalid Follower State: " + str(error))
                return
            sample = JointState()
            sample.header.stamp = message.header.stamp
            sample.name, sample.position = names, values
            self.states[side] = sample
            self.state_received[side] = time.monotonic()
            if len(self.states) != 2:
                return
            try:
                for arm_side in self.arms:
                    self.feedback.check(arm_side, self.state_timeout_s)
            except ValueError:
                return  # No combined State from unproven feedback.
            left, right = self.states["left"], self.states["right"]
            state = JointState()
            # Latest values, not a synchronized sensor sample. Use the older
            # source stamp rather than claiming that both are freshly sampled.
            state.header.stamp = min(left.header.stamp, right.header.stamp)
            state.name = [side + "_" + name for side in ("left", "right")
                          for name in ["joint%d" % i for i in range(1, 7)] + ["gripper"]]
            state.position = list(left.position) + list(right.position)
            self.state_pub.publish(state)


if __name__ == "__main__":
    VlaDualPiperController()
    rospy.spin()
