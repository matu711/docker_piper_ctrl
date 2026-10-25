#!/usr/bin/env python3
"""VLA action adapter for the existing PiPER single-arm ROS driver.

This node never opens CAN.  It delegates PiPER-specific unit conversion and
hardware commands to piper_ctrl_single_node.py:
  /joint_states (JointState) -> JointCtrl/GripperCtrl
  /pos_cmd (piper_msgs/PosCmd) -> EndPoseCtrl/GripperCtrl
"""
import math
import time

import rospy
from geometry_msgs.msg import PoseStamped
from piper_msgs.msg import PiperEulerPose, PosCmd
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray, String
from piper_sdk import C_PiperForwardKinematics
from piper_sdk.piper_param.piper_param_manager import C_PiperParamManager
from tf.transformations import euler_from_matrix, euler_matrix, quaternion_matrix


ACTION_TYPES = {
    "joint_absolute",
    "joint_delta",
    "ee_absolute",
    "ee_delta",
    "ee_delta_cumulative_reference",
    "ee_delta_physical_world_per_control_step",
}
GRIPPER_ACTION_TYPES = {
    "normalized_absolute",
    "physical_absolute",
    "pbl8_normalized_absolute",
}
PBL8_GRIPPER_SATURATION_MARGIN = 0.01


class VlaPiperController:
    def __init__(self):
        rospy.init_node("vla_piper_controller")
        self.action_type = rospy.get_param("~action_type", "joint_absolute")
        self.z_min = float(rospy.get_param("~z_min", 0.01))
        self.z_enabled = bool(rospy.get_param("~z_enabled", True))
        self.dry_run = bool(rospy.get_param("~dry_run", True))
        self.dh_is_offset = int(rospy.get_param("~dh_is_offset", 1))
        self.ee_orientation_representation = rospy.get_param(
            "~ee_orientation_representation", "euler_rpy_rad"
        )
        # The low-level PiPER driver accepts an absolute gripper opening in m.
        # A policy can instead expose the sim convention [-1, 1], where -1 is
        # open and +1 is closed, so make that contract explicit per checkpoint.
        self.gripper_action_type = rospy.get_param(
            "~gripper_action_type", "physical_absolute"
        )
        self.gripper_closed_width_m = float(rospy.get_param(
            "~gripper_closed_width_m", 0.0
        ))
        self.gripper_open_width_m = float(rospy.get_param(
            "~gripper_open_width_m", 0.10
        ))
        self.gripper_saturation_margin_m = float(rospy.get_param(
            "~gripper_saturation_margin_m", 0.01
        ))
        self.joint_saturation_margin_rad = float(rospy.get_param(
            "~joint_saturation_margin_rad", 0.05
        ))
        self._validate_action_type(self.action_type)
        self._validate_gripper_config()
        self._validate_joint_saturation_config()
        if self.ee_orientation_representation != "euler_rpy_rad":
            raise ValueError(
                "Only euler_rpy_rad is supported by the existing /pos_cmd "
                "and PiPER EndPoseCtrl route."
            )
        # Official PiPER SDK FK only. It is a pure calculation object and does
        # not open CAN; CAN ownership stays in the existing driver.
        self.fk = C_PiperForwardKinematics(dh_is_offset=self.dh_is_offset)
        self.params = C_PiperParamManager()

        self.current_joint_state = None
        self.current_ee_pose = None
        self.current_ee_euler = None
        self._current_ee_euler_received_monotonic = None
        self.hold_requested = True
        self.received_action_type = None
        self._pbl8_reference_position = None
        self._pbl8_reference_rotation = None
        self._pbl8_command_euler = None
        self.pbl8_max_translation_delta_m = float(rospy.get_param(
            "~pbl8_max_translation_delta_m", 0.003
        ))
        self.pbl8_max_rotation_delta_rad = float(rospy.get_param(
            "~pbl8_max_rotation_delta_rad", 0.01
        ))
        self.pbl8_reference_mode = rospy.get_param(
            "~pbl8_reference_mode", "cumulative"
        )
        self.pbl8_feedback_timeout_sec = float(rospy.get_param(
            "~pbl8_feedback_timeout_sec", 0.5
        ))
        self._validate_pbl8_config()

        self.joint_command_pub = rospy.Publisher(
            "/joint_states", JointState, queue_size=1
        )
        self.ee_command_pub = rospy.Publisher("/pos_cmd", PosCmd, queue_size=1)
        self.safety_state_pub = rospy.Publisher(
            "/safety/state", String, queue_size=10
        )
        self.safety_reason_pub = rospy.Publisher(
            "/safety/reason", String, queue_size=10
        )
        self.target_z_pub = rospy.Publisher(
            "/safety/target_z", Float64, queue_size=10
        )
        self.target_joint_pub = rospy.Publisher(
            "/vla/target_joint_command", JointState, queue_size=10
        )
        self.target_ee_pub = rospy.Publisher(
            "/vla/target_ee_command", PosCmd, queue_size=10
        )

        rospy.Subscriber(
            "/joint_states_single", JointState, self._joint_state_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            "/end_pose", PoseStamped, self._ee_pose_callback, queue_size=1
        )
        # Euler feedback is emitted by the existing official ROS driver and
        # avoids implementing a quaternion/Euler conversion in this node.
        rospy.Subscriber(
            "/end_pose_euler", PiperEulerPose, self._ee_euler_callback,
            queue_size=1,
        )
        # These are the actual output/hold topics of piper_vla_runtime.
        rospy.Subscriber(
            "/vla/approved_action", Float64MultiArray,
            self._action_callback, queue_size=1,
        )
        rospy.Subscriber(
            "/vla/action_type", String, self._action_type_callback,
            queue_size=1,
        )
        rospy.Subscriber("/vla/hold", Bool, self._hold_callback, queue_size=1)

        rospy.loginfo(
            "VLA PiPER controller READY: action_type=%s z_min=%.3f "
            "dry_run=%s dh_is_offset=%s gripper_action_type=%s "
            "gripper_range_m=[%.3f, %.3f] gripper_saturation_margin_m=%.3f "
            "joint_saturation_margin_rad=%.3f pbl8_reference_mode=%s "
            "pbl8_feedback_timeout_sec=%.3f",
            self.action_type, self.z_min, self.dry_run,
            self.dh_is_offset, self.gripper_action_type,
            self.gripper_closed_width_m, self.gripper_open_width_m,
            self.gripper_saturation_margin_m, self.joint_saturation_margin_rad,
            self.pbl8_reference_mode, self.pbl8_feedback_timeout_sec,
        )
        rospy.loginfo("PBL8 reference mode: %s", self.pbl8_reference_mode)

    @staticmethod
    def _validate_action_type(action_type):
        if action_type not in ACTION_TYPES:
            raise ValueError("Unsupported action_type: %s" % action_type)

    def _validate_gripper_config(self):
        if self.gripper_action_type not in GRIPPER_ACTION_TYPES:
            raise ValueError(
                "Unsupported gripper_action_type: %s" % self.gripper_action_type
            )
        if not (
            math.isfinite(self.gripper_closed_width_m)
            and math.isfinite(self.gripper_open_width_m)
        ):
            raise ValueError("Gripper widths must be finite values in meters")
        if self.gripper_closed_width_m < 0:
            raise ValueError("gripper_closed_width_m must be >= 0")
        if self.gripper_open_width_m < self.gripper_closed_width_m:
            raise ValueError(
                "gripper_open_width_m must be >= gripper_closed_width_m"
            )
        if (
            not math.isfinite(self.gripper_saturation_margin_m)
            or self.gripper_saturation_margin_m < 0.0
        ):
            raise ValueError(
                "gripper_saturation_margin_m must be finite and >= 0"
            )

    def _validate_joint_saturation_config(self):
        if (
            not math.isfinite(self.joint_saturation_margin_rad)
            or self.joint_saturation_margin_rad < 0.0
        ):
            raise ValueError(
                "joint_saturation_margin_rad must be finite and >= 0"
            )

    def _validate_pbl8_config(self):
        if self.pbl8_reference_mode not in ("cumulative", "actual_feedback"):
            raise ValueError(
                "pbl8_reference_mode must be cumulative or actual_feedback"
            )
        if (
            not math.isfinite(self.pbl8_feedback_timeout_sec)
            or self.pbl8_feedback_timeout_sec <= 0.0
        ):
            raise ValueError("pbl8_feedback_timeout_sec must be finite and > 0")

    def _gripper_action_to_width_m(self, value):
        """Convert one policy gripper action into an absolute PiPER width [m]."""
        value = float(value)
        if not math.isfinite(value):
            raise ValueError("gripper action must be finite")
        if self.gripper_action_type == "normalized_absolute":
            normalized = min(1.0, max(-1.0, value))
            return self.gripper_closed_width_m + (
                (1.0 - normalized) / 2.0
            ) * (self.gripper_open_width_m - self.gripper_closed_width_m)
        if self.gripper_action_type == "pbl8_normalized_absolute":
            if (
                value < -1.0 - PBL8_GRIPPER_SATURATION_MARGIN
                or value > 1.0 + PBL8_GRIPPER_SATURATION_MARGIN
            ):
                raise ValueError(
                    "PBL8 gripper command outside saturation margin "
                    "[-1.01, 1.01]: %.9f" % value
                )
            applied = min(1.0, max(-1.0, value))
            if applied != value:
                rospy.logwarn_throttle(
                    1.0,
                    "PBL8 gripper saturated: raw=%.9f applied=%.9f "
                    "normalized_range=[-1.000000, 1.000000]",
                    value, applied,
                )
            # PBL8 sim semantics: -1 is fully open, 0 is the closed
            # equilibrium, and positive commands continue closing force.
            opening_fraction = max(0.0, -applied)
            return self.gripper_closed_width_m + opening_fraction * (
                self.gripper_open_width_m - self.gripper_closed_width_m
            )
        allowed_lower = (
            self.gripper_closed_width_m - self.gripper_saturation_margin_m
        )
        allowed_upper = (
            self.gripper_open_width_m + self.gripper_saturation_margin_m
        )
        if value < allowed_lower or value > allowed_upper:
            raise ValueError(
                "gripper_outside_saturation_margin raw=%.6f "
                "allowed_range=[%.6f, %.6f]" % (
                    value, allowed_lower, allowed_upper
                )
            )
        applied = min(
            self.gripper_open_width_m,
            max(self.gripper_closed_width_m, value),
        )
        if applied != value:
            rospy.logwarn_throttle(
                1.0,
                "Gripper saturated:\nraw=%.4f m\napplied=%.4f m\n"
                "physical_range=[%.4f, %.4f] m",
                value, applied, self.gripper_closed_width_m,
                self.gripper_open_width_m,
            )
        return applied

    def _joint_state_callback(self, message):
        self.current_joint_state = message

    def _ee_pose_callback(self, message):
        self.current_ee_pose = message

    def _ee_euler_callback(self, message):
        self.current_ee_euler = message
        self._current_ee_euler_received_monotonic = time.monotonic()

    def _action_type_callback(self, message):
        self._validate_action_type(message.data)
        self.received_action_type = message.data

    def _hold_callback(self, message):
        self.hold_requested = message.data
        if message.data:
            self._hold("hold_requested")

    def _hold(self, reason):
        self._reset_pbl8_reference()
        self.safety_state_pub.publish("HOLD")
        self.safety_reason_pub.publish(reason)
        rospy.logwarn_throttle(1.0, "VLA action held: %s", reason)

    @staticmethod
    def _require_action_size(action):
        if len(action) < 7:
            raise ValueError("Expected action [0:7], got %d values" % len(action))

    @staticmethod
    def _require_finite_action(action):
        if not all(math.isfinite(float(value)) for value in action[:7]):
            raise ValueError("Action [0:7] must contain only finite values")

    def _action_callback(self, message):
        if self.hold_requested:
            self._hold("hold_requested")
            return
        if self.received_action_type and self.received_action_type != self.action_type:
            self._hold(
                "action_type_mismatch runtime=%s controller=%s" % (
                    self.received_action_type, self.action_type
                )
            )
            return
        try:
            self._require_action_size(message.data)
            self._require_finite_action(message.data)
            handler = {
                "joint_absolute": self.handle_joint_absolute,
                "joint_delta": self.handle_joint_delta,
                "ee_absolute": self.handle_ee_absolute,
                "ee_delta": self.handle_ee_delta,
                "ee_delta_cumulative_reference": self.handle_pbl8_ee_delta,
                "ee_delta_physical_world_per_control_step": self.handle_pbl8_ee_delta,
            }[self.action_type]
            handler(list(message.data[:7]))
        except (RuntimeError, ValueError) as error:
            self._hold(str(error))

    def handle_joint_absolute(self, action):
        self._send_joint_target(action, "joint_absolute")

    def handle_joint_delta(self, action):
        if self.current_joint_state is None or len(self.current_joint_state.position) < 7:
            raise RuntimeError("current /joint_states_single is unavailable")
        target = [
            current + delta
            for current, delta in zip(self.current_joint_state.position[:7], action)
        ]
        self._send_joint_target(target, "joint_delta")

    def handle_ee_absolute(self, action):
        command = self._make_ee_command(action)
        self._check_ee_z_and_send(command, "ee_absolute")

    def handle_ee_delta(self, action):
        if self.current_ee_pose is None or self.current_ee_euler is None:
            raise RuntimeError(
                "current /end_pose and /end_pose_euler are required for ee_delta"
            )
        # No VLA bundle in this repository defines the representation of
        # drx/dry/drz.  Applying them as Euler additions would be an invented
        # rotation rule, so nonzero rotational deltas deliberately fail closed.
        if any(abs(value) > 1e-12 for value in action[3:6]):
            raise RuntimeError(
                "ee_delta rotational components are unsupported until the "
                "checkpoint manifest declares their representation"
            )
        current = self.current_ee_pose.pose.position
        command = PosCmd()
        command.x = current.x + action[0]
        command.y = current.y + action[1]
        command.z = current.z + action[2]
        # Use Euler feedback produced by piper_ctrl_single_node.py directly.
        command.roll = self.current_ee_euler.roll
        command.pitch = self.current_ee_euler.pitch
        command.yaw = self.current_ee_euler.yaw
        command.gripper = self._gripper_action_to_width_m(action[6])
        self._check_ee_z_and_send(command, "ee_delta")

    def _reset_pbl8_reference(self):
        self._pbl8_reference_position = None
        self._pbl8_reference_rotation = None
        self._pbl8_command_euler = None

    @staticmethod
    def _unwrap_euler_near_reference(raw_euler, reference_euler):
        """Choose each Euler component's nearest equivalent 2*pi branch."""
        if len(raw_euler) != 3 or len(reference_euler) != 3:
            raise ValueError("Euler unwrap requires three raw and reference angles")
        command_euler = []
        for raw_angle, reference_angle in zip(raw_euler, reference_euler):
            angle = float(raw_angle)
            reference = float(reference_angle)
            if not math.isfinite(angle) or not math.isfinite(reference):
                raise ValueError("Euler unwrap angles must be finite")
            while angle - reference > math.pi:
                angle -= 2.0 * math.pi
            while angle - reference < -math.pi:
                angle += 2.0 * math.pi
            command_euler.append(angle)
        return command_euler

    @staticmethod
    def _rotation_vector_to_matrix(rotation_vector):
        vector = [float(value) for value in rotation_vector]
        angle = math.sqrt(sum(value * value for value in vector))
        if angle < 1e-15:
            return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        axis = [value / angle for value in vector]
        x, y, z = axis
        cosine = math.cos(angle)
        sine = math.sin(angle)
        one_minus_cosine = 1.0 - cosine
        return [
            [cosine + x * x * one_minus_cosine,
             x * y * one_minus_cosine - z * sine,
             x * z * one_minus_cosine + y * sine],
            [y * x * one_minus_cosine + z * sine,
             cosine + y * y * one_minus_cosine,
             y * z * one_minus_cosine - x * sine],
            [z * x * one_minus_cosine - y * sine,
             z * y * one_minus_cosine + x * sine,
             cosine + z * z * one_minus_cosine],
        ]

    @staticmethod
    def _matrix_multiply(left, right):
        return [
            [sum(left[row][k] * right[k][column] for k in range(3))
             for column in range(3)]
            for row in range(3)
        ]

    def _initialize_pbl8_reference(self):
        if self.current_ee_pose is None or self.current_ee_euler is None:
            raise RuntimeError(
                "current /end_pose and /end_pose_euler are required for PBL8"
            )
        pose = self.current_ee_pose.pose
        quaternion = [
            float(pose.orientation.x), float(pose.orientation.y),
            float(pose.orientation.z), float(pose.orientation.w),
        ]
        quaternion_norm = math.sqrt(sum(value * value for value in quaternion))
        if (
            not all(math.isfinite(value) for value in quaternion)
            or quaternion_norm < 1e-12
        ):
            raise RuntimeError("current /end_pose quaternion is invalid")
        quaternion = [value / quaternion_norm for value in quaternion]
        self._pbl8_reference_position = [
            float(pose.position.x), float(pose.position.y), float(pose.position.z)
        ]
        matrix4 = quaternion_matrix(quaternion)
        self._pbl8_reference_rotation = [
            [float(matrix4[row][column]) for column in range(3)]
            for row in range(3)
        ]
        self._pbl8_command_euler = [
            float(self.current_ee_euler.roll),
            float(self.current_ee_euler.pitch),
            float(self.current_ee_euler.yaw),
        ]
        if not all(math.isfinite(value) for value in self._pbl8_command_euler):
            raise RuntimeError("current /end_pose_euler is invalid")

    def _latest_pbl8_actual_feedback(self):
        feedback = self.current_ee_euler
        received = self._current_ee_euler_received_monotonic
        if feedback is None or received is None:
            raise RuntimeError("pbl8_ee_feedback_stale feedback_unavailable")
        age = time.monotonic() - received
        if age < 0.0 or age > self.pbl8_feedback_timeout_sec:
            raise RuntimeError(
                "pbl8_ee_feedback_stale age=%.6f timeout=%.6f"
                % (age, self.pbl8_feedback_timeout_sec)
            )
        values = [
            float(feedback.x), float(feedback.y), float(feedback.z),
            float(feedback.roll), float(feedback.pitch), float(feedback.yaw),
        ]
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError("pbl8_ee_feedback_stale feedback_nonfinite")
        return values

    @staticmethod
    def _rotation_matrix_from_euler(roll, pitch, yaw):
        matrix4 = euler_matrix(roll, pitch, yaw, axes="sxyz")
        return [
            [float(matrix4[row][column]) for column in range(3)]
            for row in range(3)
        ]

    def handle_pbl8_ee_delta(self, action):
        """Execute a PBL8 world-frame delta using the configured reference."""
        translation = [float(value) for value in action[:3]]
        rotation_vector = [float(value) for value in action[3:6]]
        translation_norm = math.sqrt(sum(value * value for value in translation))
        rotation_norm = math.sqrt(sum(value * value for value in rotation_vector))
        if translation_norm > self.pbl8_max_translation_delta_m + 1e-8:
            raise RuntimeError(
                "pbl8_translation_delta_limit norm=%.9f limit=%.9f" % (
                    translation_norm, self.pbl8_max_translation_delta_m
                )
            )
        if rotation_norm > self.pbl8_max_rotation_delta_rad + 1e-8:
            raise RuntimeError(
                "pbl8_rotation_delta_limit norm=%.9f limit=%.9f" % (
                    rotation_norm, self.pbl8_max_rotation_delta_rad
                )
            )
        if self.pbl8_reference_mode == "cumulative":
            if self._pbl8_reference_position is None:
                self._initialize_pbl8_reference()
            base_position = self._pbl8_reference_position
            base_rotation = self._pbl8_reference_rotation
            unwrap_reference = self._pbl8_command_euler
        else:
            feedback = self._latest_pbl8_actual_feedback()
            base_position = feedback[:3]
            unwrap_reference = feedback[3:6]
            base_rotation = self._rotation_matrix_from_euler(
                *unwrap_reference
            )

        candidate_position = [
            current + delta
            for current, delta in zip(base_position, translation)
        ]
        candidate_rotation = self._matrix_multiply(
            self._rotation_vector_to_matrix(rotation_vector),
            base_rotation,
        )
        raw_euler = list(euler_from_matrix(candidate_rotation, axes="sxyz"))
        command_euler = self._unwrap_euler_near_reference(
            raw_euler, unwrap_reference
        )
        for axis, raw_angle, reference_angle, command_angle in zip(
            ("roll", "pitch", "yaw"), raw_euler,
            unwrap_reference, command_euler,
        ):
            if command_angle != raw_angle:
                rospy.logwarn_throttle(
                    1.0,
                    "PBL8 Euler unwrap:\n%s raw=%.9f\n%s reference=%.9f\n"
                    "%s command=%.9f",
                    axis, raw_angle, axis, reference_angle,
                    axis, command_angle,
                )
        roll, pitch, yaw = command_euler
        command = PosCmd()
        command.x, command.y, command.z = candidate_position
        command.roll = float(roll)
        command.pitch = float(pitch)
        command.yaw = float(yaw)
        command.gripper = self._gripper_action_to_width_m(action[6])
        self._check_ee_z_and_send(command, self.action_type)
        if self.pbl8_reference_mode == "cumulative":
            self._pbl8_reference_position = candidate_position
            self._pbl8_reference_rotation = candidate_rotation
            self._pbl8_command_euler = command_euler
        else:
            rospy.loginfo_throttle(
                1.0,
                "PBL8 actual-feedback: actual_xyz=[%.4f, %.4f, %.4f] "
                "delta_xyz=[%.4f, %.4f, %.4f] "
                "target_xyz=[%.4f, %.4f, %.4f]",
                base_position[0], base_position[1], base_position[2],
                translation[0], translation[1], translation[2],
                candidate_position[0], candidate_position[1],
                candidate_position[2],
            )

    def _make_ee_command(self, action):
        command = PosCmd()
        command.x, command.y, command.z = action[:3]
        command.roll, command.pitch, command.yaw = action[3:6]
        command.gripper = self._gripper_action_to_width_m(action[6])
        return command

    def _send_joint_target(self, target, source):
        applied_target = self._apply_joint_saturation(target)
        # FK and the Z gate intentionally use the physical, saturated target.
        target_z = self._official_target_joint_z(applied_target)
        self.target_z_pub.publish(target_z)
        if self.z_enabled and target_z < self.z_min:
            raise RuntimeError(
                "target_z_below_min target_z=%.6f z_min=%.6f" % (
                    target_z, self.z_min
                )
            )
        raw_gripper = float(applied_target[6])
        applied_gripper = self._gripper_action_to_width_m(raw_gripper)
        applied_target[6] = applied_gripper
        command = JointState()
        command.header.stamp = rospy.Time.now()
        command.name = ["joint%d" % index for index in range(1, 7)] + ["gripper"]
        command.position = applied_target
        self.target_joint_pub.publish(command)
        if self.dry_run:
            rospy.loginfo(
                "DRY RUN gripper: raw=%.6f m applied=%.6f m safety=DRY_RUN",
                raw_gripper, applied_gripper,
            )
        self._publish_or_dry_run(self.joint_command_pub, command, source)

    def _official_target_joint_z(self, target):
        if len(target) < 6:
            raise ValueError("joint target has fewer than 6 arm joints")
        # C_PiperForwardKinematics.CalFK is the official SDK implementation;
        # it consumes radian joints and returns XYZ in mm.  See piper_sdk
        # kinematics/piper_fk.py and interface GetFK documentation.
        pose_mm_deg = self.fk.CalFK(target[:6])[-1]
        return float(pose_mm_deg[2]) / 1000.0

    def _apply_joint_saturation(self, target):
        """Return a physical joint target, clipping only inside the margin."""
        if len(target) < 7:
            raise ValueError("joint target has fewer than 7 values")
        applied_target = list(target)
        margin = self.joint_saturation_margin_rad
        for index, raw_value in enumerate(target[:6], start=1):
            value = float(raw_value)
            if not math.isfinite(value):
                raise ValueError("joint%d target is not finite" % index)
            lower, upper = self.params.GetJointLimitParam("j%d" % index)
            accepted_lower, accepted_upper = lower - margin, upper + margin
            if value < accepted_lower:
                raise RuntimeError(
                    "joint%d_limit_violation target=%.6f accepted_min=%.6f "
                    "physical_min=%.6f" % (
                        index, value, accepted_lower, lower
                    )
                )
            if value > accepted_upper:
                raise RuntimeError(
                    "joint%d_limit_violation target=%.6f accepted_max=%.6f "
                    "physical_max=%.6f" % (
                        index, value, accepted_upper, upper
                    )
                )
            saturated = min(upper, max(lower, value))
            if saturated != value:
                rospy.logwarn_throttle(
                    1.0,
                    "Joint saturated:\n"
                    "joint=%d\nraw=%.6f rad\napplied=%.6f rad\n"
                    "physical_range=[%.6f, %.6f] rad\nmargin=%.6f rad",
                    index, value, saturated, lower, upper, margin,
                )
            applied_target[index - 1] = saturated
        return applied_target

    def _check_ee_z_and_send(self, command, source):
        self.target_z_pub.publish(command.z)
        if self.z_enabled and command.z < self.z_min:
            raise RuntimeError(
                "target_z_below_min target_z=%.6f z_min=%.6f" % (
                    command.z, self.z_min
                )
            )
        self.target_ee_pub.publish(command)
        self._publish_or_dry_run(self.ee_command_pub, command, source)

    def _publish_or_dry_run(self, publisher, command, source):
        if self.dry_run:
            self.safety_state_pub.publish("DRY_RUN")
            # `source` identifies the action path for diagnostics; it is not a
            # safety failure reason. Successful actions always clear reason.
            self.safety_reason_pub.publish("")
            rospy.loginfo("DRY RUN %s: %s", source, command)
            return
        self.safety_state_pub.publish("OK")
        self.safety_reason_pub.publish("")
        publisher.publish(command)


if __name__ == "__main__":
    VlaPiperController()
    rospy.spin()
