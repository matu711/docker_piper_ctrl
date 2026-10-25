#!/usr/bin/env python3
"""Return PiPER to the YAML initial pose through exactly one command owner.

If piper_ctrl_single_node is active, commands go directly to that driver's
JointState input.  Otherwise this process becomes the sole can0 owner and uses
the official Piper SDK.  The VLA action path is never used.
"""

import math
import os
import threading
import time
from collections.abc import Mapping

import rospy
import yaml
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


TARGET_KEYS = (
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"
)
STATE_NAMES = list(TARGET_KEYS)
FEEDBACK_TOPIC = "/joint_states_single"
DRIVER_COMMAND_TOPICS = ("/joint_states", "/joint_ctrl_single")
DRIVER_EVIDENCE_PUBLISH_TOPICS = {
    FEEDBACK_TOPIC, "/arm_status", "/end_pose", "/end_pose_euler",
    "/current_joint_obs",
}
DRIVER_EVIDENCE_SUBSCRIBE_TOPICS = {
    "/joint_states", "/joint_ctrl_single", "/pos_cmd", "/enable_flag",
}
DRIVER_EVIDENCE_SERVICES = {
    "/enable_srv", "/stop_srv", "/reset_srv", "/go_zero_srv",
    "/block_arm", "/go_initial_pose",
}


def _looks_like_piper_can_owner(node):
    lowered = node.lower()
    return any(name in lowered for name in (
        "piper_ctrl_single_node",
        "piper_ctrl_follower_node",
        "piper_ctrl_leader_node",
    ))


def validate_target(value):
    if not isinstance(value, Mapping):
        raise ValueError("config target must be a mapping")
    missing = [name for name in TARGET_KEYS if name not in value]
    extra = [name for name in value if name not in TARGET_KEYS]
    if missing or extra:
        raise ValueError(
            "target keys must be exactly %s; missing=%s extra=%s"
            % (list(TARGET_KEYS), missing, extra)
        )
    target = [float(value[name]) for name in TARGET_KEYS]
    if not all(math.isfinite(item) for item in target):
        raise ValueError("target contains a non-finite value")
    return target


def load_initial_pose_config(path):
    with open(path, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, Mapping):
        raise ValueError("initial-pose YAML must be a mapping")
    if "target" not in config:
        raise ValueError("initial-pose YAML has no target")
    target = validate_target(config["target"])
    return config, target


def validate_state(values):
    if len(values) < 7:
        raise ValueError("joint feedback has fewer than seven positions")
    state = [float(value) for value in values[:7]]
    if not all(math.isfinite(value) for value in state):
        raise ValueError("joint feedback contains a non-finite value")
    return state


def smoothstep_trajectory(start, target, total_steps):
    if total_steps <= 0:
        raise ValueError("total_steps must be positive")
    if len(start) != 7 or len(target) != 7:
        raise ValueError("trajectory endpoints must have seven values")
    trajectory = []
    for step in range(1, total_steps + 1):
        if step == total_steps:
            trajectory.append(list(target))
            continue
        ratio = float(step) / float(total_steps)
        blend = ratio * ratio * (3.0 - 2.0 * ratio)
        trajectory.append([
            current + blend * (goal - current)
            for current, goal in zip(start, target)
        ])
    return trajectory


def validate_pose_limits(target, parameter_manager, gripper_max_width_m=0.10):
    target = validate_state(target)
    for index, value in enumerate(target[:6], start=1):
        lower, upper = parameter_manager.GetJointLimitParam("j%d" % index)
        if value < lower or value > upper:
            raise ValueError(
                "joint%d %.6f rad outside official limit [%.6f, %.6f]"
                % (index, value, lower, upper)
            )
    if target[6] < 0.0 or target[6] > gripper_max_width_m:
        raise ValueError(
            "gripper %.6f m outside [0.000000, %.6f]"
            % (target[6], gripper_max_width_m)
        )
    return target


def analyze_driver_topology(system_state):
    """Return (command_topic, driver_node, evidence) from ROS master state."""
    publishers, subscribers, services = system_state
    feedback_nodes = set()
    evidence_nodes = set()
    for topic, nodes in publishers:
        evidence_nodes.update(node for node in nodes if _looks_like_piper_can_owner(node))
        if topic == FEEDBACK_TOPIC:
            feedback_nodes.update(nodes)
        if topic in DRIVER_EVIDENCE_PUBLISH_TOPICS:
            evidence_nodes.update(nodes)

    routes = set()
    driver_named_subscribers = set()
    for topic, nodes in subscribers:
        evidence_nodes.update(node for node in nodes if _looks_like_piper_can_owner(node))
        if topic not in DRIVER_COMMAND_TOPICS:
            pass
        else:
            for node in nodes:
                if node in feedback_nodes:
                    routes.add((topic, node))
                if "piper_ctrl_single_node" in node:
                    driver_named_subscribers.add(node)
        if topic in DRIVER_EVIDENCE_SUBSCRIBE_TOPICS:
            evidence_nodes.update(
                node for node in nodes
                if "piper" in node.lower() and "go_initial_pose" not in node
            )

    for service, nodes in services:
        evidence_nodes.update(node for node in nodes if _looks_like_piper_can_owner(node))
        if service in DRIVER_EVIDENCE_SERVICES:
            evidence_nodes.update(
                node for node in nodes if "go_initial_pose" not in node
            )

    if len(routes) > 1:
        raise RuntimeError("Ambiguous PiPER driver routes: %s" % sorted(routes))
    if routes:
        topic, node = next(iter(routes))
        return topic, node, True
    evidence = bool(
        feedback_nodes or driver_named_subscribers or evidence_nodes
    )
    return None, None, evidence


class JointFeedbackMonitor:
    def __init__(self):
        self.condition = threading.Condition()
        self.state = None
        self.error = None
        self.received_monotonic = None

    def callback(self, message):
        try:
            state, error = validate_state(message.position), None
        except ValueError as caught:
            state, error = None, str(caught)
        with self.condition:
            self.state = state
            self.error = error
            self.received_monotonic = time.monotonic()
            self.condition.notify_all()

    def wait(self, timeout):
        deadline = time.monotonic() + timeout
        with self.condition:
            while not rospy.is_shutdown():
                if self.error:
                    raise RuntimeError(self.error)
                if self.state is not None:
                    return list(self.state)
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise RuntimeError(
                        "Timed out waiting for %s after %.1f s"
                        % (FEEDBACK_TOPIC, timeout)
                    )
                self.condition.wait(min(remaining, 0.1))
        raise RuntimeError("ROS shutdown while waiting for joint feedback")

    def latest(self, maximum_age):
        with self.condition:
            if self.error:
                raise RuntimeError(self.error)
            if self.state is None or self.received_monotonic is None:
                raise RuntimeError("PiPER joint feedback is unavailable")
            age = time.monotonic() - self.received_monotonic
            if age > maximum_age:
                raise RuntimeError(
                    "PiPER joint feedback is stale: age=%.3f timeout=%.3f"
                    % (age, maximum_age)
                )
            return list(self.state)


class GoInitialPose:
    def __init__(self):
        rospy.init_node("go_initial_pose")
        default_config = os.path.abspath(os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            "..", "config", "piper_demo_start.yaml",
        ))
        self.config_file = os.path.abspath(os.path.expanduser(
            rospy.get_param("~config_file", default_config)
        ))
        config, self.target = load_initial_pose_config(self.config_file)
        self.move_duration = float(rospy.get_param(
            "~move_duration", config.get("move_duration", 20.0)
        ))
        self.command_rate = float(rospy.get_param(
            "~command_rate", config.get("command_rate", 10.0)
        ))
        self.detection_timeout = float(rospy.get_param(
            "~driver_detection_timeout", 2.0
        ))
        self.feedback_timeout = float(rospy.get_param("~feedback_timeout", 1.0))
        self.arrival_timeout = float(rospy.get_param("~arrival_timeout", 10.0))
        self.sdk_enable_timeout = float(rospy.get_param(
            "~sdk_enable_timeout", 10.0
        ))
        self.joint_tolerance = float(rospy.get_param(
            "~joint_tolerance_rad", 0.02
        ))
        self.gripper_tolerance = float(rospy.get_param(
            "~gripper_tolerance_m", 0.002
        ))
        self.direct_speed = int(rospy.get_param("~direct_speed_percent", 10))
        self.can_port = rospy.get_param("~can_port", "can0")
        self.gripper_max_width_m = float(rospy.get_param(
            "~gripper_max_width_m", 0.10
        ))
        self.z_enabled = bool(rospy.get_param("~z_enabled", True))
        self.z_min = float(rospy.get_param("~z_min", 0.01))
        self._validate_parameters()

        self.hold_pub = rospy.Publisher("/vla/hold", Bool, queue_size=1, latch=True)
        self.feedback = JointFeedbackMonitor()
        rospy.Subscriber(
            FEEDBACK_TOPIC, JointState, self.feedback.callback, queue_size=1
        )
        self.hold_pub.publish(Bool(data=True))
        self.hold_timer = rospy.Timer(
            rospy.Duration(0.1), self._keep_vla_held
        )
        rospy.on_shutdown(self._keep_vla_held)

    def _validate_parameters(self):
        positive = {
            "move_duration": self.move_duration,
            "command_rate": self.command_rate,
            "driver_detection_timeout": self.detection_timeout,
            "feedback_timeout": self.feedback_timeout,
            "arrival_timeout": self.arrival_timeout,
            "sdk_enable_timeout": self.sdk_enable_timeout,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("~%s must be finite and > 0" % name)
        for name, value in (
            ("joint_tolerance_rad", self.joint_tolerance),
            ("gripper_tolerance_m", self.gripper_tolerance),
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError("~%s must be finite and >= 0" % name)
        if not 1 <= self.direct_speed <= 100:
            raise ValueError("~direct_speed_percent must be in [1, 100]")
        if not 0.0 < self.gripper_max_width_m <= 0.10:
            raise ValueError("~gripper_max_width_m must be in (0, 0.10]")
        if not math.isfinite(self.z_min):
            raise ValueError("~z_min must be finite")

    def _keep_vla_held(self, _event=None):
        try:
            self.hold_pub.publish(Bool(data=True))
        except Exception:
            pass

    @staticmethod
    def _system_state():
        import rosgraph
        return rosgraph.Master(rospy.get_name()).getSystemState()

    def _detect_backend(self):
        deadline = time.monotonic() + self.detection_timeout
        evidence_seen = False
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            topic, node, evidence = analyze_driver_topology(self._system_state())
            evidence_seen = evidence_seen or evidence
            if topic:
                rospy.loginfo(
                    "PiPER backend auto-detected: ROS driver node=%s command=%s",
                    node, topic,
                )
                return "ros_driver", topic
            rospy.sleep(0.05)
        if evidence_seen:
            raise RuntimeError(
                "PiPER ROS driver evidence was detected, but a complete feedback/"
                "command route was not ready; refusing direct CAN access"
            )
        rospy.loginfo(
            "No PiPER ROS driver detected for %.1f s; selecting direct SDK on %s",
            self.detection_timeout, self.can_port,
        )
        return "direct_sdk", None

    @staticmethod
    def _sdk_dependencies():
        from piper_sdk import C_PiperForwardKinematics, Piper
        from piper_sdk.piper_param.piper_param_manager import C_PiperParamManager
        return Piper, C_PiperParamManager, C_PiperForwardKinematics

    def _make_trajectory(self, start, parameter_manager, fk):
        start = validate_pose_limits(
            start, parameter_manager, self.gripper_max_width_m
        )
        target = validate_pose_limits(
            self.target, parameter_manager, self.gripper_max_width_m
        )
        total_steps = int(math.ceil(self.move_duration * self.command_rate))
        trajectory = smoothstep_trajectory(start, target, total_steps)
        for step, point in enumerate(trajectory, start=1):
            validate_pose_limits(point, parameter_manager, self.gripper_max_width_m)
            target_z = float(fk.CalFK(point[:6])[-1][2]) / 1000.0
            if not math.isfinite(target_z):
                raise RuntimeError("FK returned non-finite Z at step %d" % step)
            if self.z_enabled and target_z < self.z_min:
                raise RuntimeError(
                    "FK Z safety failed before motion: step=%d target_z=%.6f "
                    "z_min=%.6f" % (step, target_z, self.z_min)
                )
        return trajectory

    @staticmethod
    def _joint_message(point, speed):
        message = JointState()
        message.header.stamp = rospy.Time.now()
        message.name = STATE_NAMES
        message.position = list(point)
        message.velocity = [0.0] * 6 + [float(speed)]
        message.effort = [0.0] * 6 + [1.0]
        return message

    def _sleep_until(self, deadline):
        while not rospy.is_shutdown():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return
            rospy.sleep(min(remaining, 0.05))
        raise RuntimeError("ROS shutdown during initial-pose trajectory")

    def _errors(self, actual):
        joint_error = max(
            abs(current - target)
            for current, target in zip(actual[:6], self.target[:6])
        )
        gripper_error = abs(actual[6] - self.target[6])
        return joint_error, gripper_error

    def _wait_driver_arrival(self):
        deadline = time.monotonic() + self.arrival_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            actual = self.feedback.latest(self.feedback_timeout)
            joint_error, gripper_error = self._errors(actual)
            if (
                joint_error <= self.joint_tolerance
                and gripper_error <= self.gripper_tolerance
            ):
                return actual
            rospy.loginfo_throttle(
                1.0, "Waiting for initial pose: joint_error=%.6f rad "
                "gripper_error=%.6f m", joint_error, gripper_error,
            )
            rospy.sleep(0.05)
        raise RuntimeError("Timed out waiting for PiPER to reach initial pose")

    def _run_ros_driver(self, command_topic, parameter_manager, fk):
        start = self.feedback.wait(self.feedback_timeout)
        trajectory = self._make_trajectory(start, parameter_manager, fk)
        publisher = rospy.Publisher(command_topic, JointState, queue_size=1)
        connection_deadline = time.monotonic() + self.feedback_timeout
        while publisher.get_num_connections() == 0:
            if time.monotonic() >= connection_deadline:
                raise RuntimeError("ROS driver command subscriber disappeared")
            rospy.sleep(0.05)

        rospy.loginfo(
            "Returning through ROS driver: duration=%.1f s rate=%.1f Hz "
            "steps=%d start=%s target=%s",
            self.move_duration, self.command_rate, len(trajectory), start,
            self.target,
        )
        try:
            started = time.monotonic()
            for index, point in enumerate(trajectory, start=1):
                self._sleep_until(started + index / self.command_rate)
                self.feedback.latest(self.feedback_timeout)
                self._keep_vla_held()
                publisher.publish(self._joint_message(point, self.direct_speed))
            return self._wait_driver_arrival()
        except Exception:
            # A position command at the last measured pose prevents the driver
            # from continuing toward a now-abandoned later interpolation point.
            try:
                actual = self.feedback.latest(self.feedback_timeout)
                publisher.publish(
                    self._joint_message(actual, self.direct_speed)
                )
                rospy.logwarn("Sent ROS-driver position HOLD at latest feedback")
            except Exception as hold_error:
                rospy.logerr("Could not send ROS-driver position HOLD: %s", hold_error)
            raise

    def _ensure_no_ros_driver(self):
        topic, node, evidence = analyze_driver_topology(self._system_state())
        if topic or evidence:
            raise RuntimeError(
                "PiPER ROS driver appeared while direct SDK mode was selected; "
                "stopping direct commands (node=%s topic=%s)" % (node, topic)
            )

    @staticmethod
    def _read_sdk_state(piper):
        joints = list(piper.get_joint_states()[0])
        gripper = float(piper.get_gripper_states()[0][0])
        return validate_state(joints + [gripper])

    def _wait_sdk_arrival(self, piper):
        deadline = time.monotonic() + self.arrival_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self._ensure_no_ros_driver()
            actual = self._read_sdk_state(piper)
            joint_error, gripper_error = self._errors(actual)
            if (
                joint_error <= self.joint_tolerance
                and gripper_error <= self.gripper_tolerance
            ):
                return actual
            rospy.loginfo_throttle(
                1.0, "Waiting for direct-SDK initial pose: "
                "joint_error=%.6f rad gripper_error=%.6f m",
                joint_error, gripper_error,
            )
            time.sleep(0.05)
        raise RuntimeError("Timed out waiting for direct-SDK initial pose")

    def _run_direct_sdk(self, Piper, parameter_manager, fk):
        # Recheck immediately before opening can0.  Never fall through to CAN
        # when a ROS-side indication of an owner exists.
        self._ensure_no_ros_driver()
        piper = Piper(self.can_port)
        interface = piper.init()
        piper.connect()
        time.sleep(0.3)
        start = self._read_sdk_state(piper)
        trajectory = self._make_trajectory(start, parameter_manager, fk)

        enable_deadline = time.monotonic() + self.sdk_enable_timeout
        while not piper.enable_arm():
            if time.monotonic() >= enable_deadline:
                raise RuntimeError("Timed out enabling PiPER arm through SDK")
            time.sleep(0.02)
        piper.enable_gripper()
        mode_deadline = time.monotonic() + self.sdk_enable_timeout
        while interface.GetArmStatus().arm_status.ctrl_mode != 1:
            if time.monotonic() >= mode_deadline:
                raise RuntimeError(
                    "Timed out switching PiPER to CAN mode; no command sent"
                )
            interface.ModeCtrl(0x01, 0x01, self.direct_speed, 0x00)
            time.sleep(0.02)

        rospy.loginfo(
            "Returning through direct Piper SDK: can=%s speed=%d%% "
            "duration=%.1f s rate=%.1f Hz steps=%d start=%s target=%s",
            self.can_port, self.direct_speed, self.move_duration,
            self.command_rate, len(trajectory), start, self.target,
        )
        try:
            started = time.monotonic()
            for index, point in enumerate(trajectory, start=1):
                self._sleep_until(started + index / self.command_rate)
                self._ensure_no_ros_driver()
                self._keep_vla_held()
                piper.move_j(point[:6], self.direct_speed)
                piper.move_gripper(point[6], 1)
            return self._wait_sdk_arrival(piper)
        except Exception:
            # Do not issue even a hold command if a ROS driver has appeared;
            # that process now owns CAN and direct access must cease instantly.
            try:
                self._ensure_no_ros_driver()
                actual = self._read_sdk_state(piper)
                piper.move_j(actual[:6], self.direct_speed)
                piper.move_gripper(actual[6], 1)
                rospy.logwarn("Sent direct-SDK position HOLD at latest feedback")
            except Exception as hold_error:
                rospy.logerr("Direct-SDK position HOLD was not sent: %s", hold_error)
            raise

    def run(self):
        self.hold_pub.publish(Bool(data=True))
        rospy.sleep(0.2)
        Piper, ParameterManager, ForwardKinematics = self._sdk_dependencies()
        parameter_manager = ParameterManager()
        fk = ForwardKinematics(dh_is_offset=1)
        # Validate the YAML target before selecting or opening either backend.
        validate_pose_limits(
            self.target, parameter_manager, self.gripper_max_width_m
        )
        backend, command_topic = self._detect_backend()
        try:
            if backend == "ros_driver":
                actual = self._run_ros_driver(
                    command_topic, parameter_manager, fk
                )
            else:
                actual = self._run_direct_sdk(
                    Piper, parameter_manager, fk
                )
        finally:
            self._keep_vla_held()
        error = [
            current - target
            for current, target in zip(actual, self.target)
        ]
        rospy.loginfo(
            "Initial pose reached through %s.\nactual=%s\ntarget=%s\nerror=%s\n"
            "VLA remains HOLD.", backend, actual, self.target, error,
        )


if __name__ == "__main__":
    try:
        GoInitialPose().run()
    except (OSError, RuntimeError, ValueError, yaml.YAMLError) as error:
        rospy.logerr("Initial-pose return stopped: %s", error)
        raise SystemExit(1)
