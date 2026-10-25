#!/usr/bin/env python3
"""Unit tests for vla_piper_controller without ROS, SDK hardware, or CAN."""

import importlib.util
import math
import pathlib
import sys
import time
import types
import unittest


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeJointState:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None)
        self.name = []
        self.position = []


class FakePosCmd:
    pass


class FakePoseStamped:
    def __init__(self):
        self.pose = types.SimpleNamespace(
            position=types.SimpleNamespace(x=0.4, y=0.0, z=0.3),
            orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )


class FakeFk:
    def CalFK(self, _joints):
        return [[0.0, 0.0, 200.0]]


class FakeFkChecksAppliedJoint:
    def __init__(self):
        self.calls = []

    def CalFK(self, joints):
        self.calls.append(list(joints))
        # The raw test target is 1.23 rad. Only a clipped 1.22 rad target
        # is safely above Z min; this proves FK receives the applied target.
        z_mm = 200.0 if joints[4] <= 1.22 else 10.0
        return [[0.0, 0.0, z_mm]]


class FakeParams:
    def __init__(self, limits=None):
        self.limits = limits or {}

    def GetJointLimitParam(self, joint_name):
        return self.limits.get(joint_name, (-3.0, 3.0))


class FakeJoint5Params(FakeParams):
    def __init__(self):
        super().__init__({"j5": (-1.22, 1.22)})


def load_controller_module():
    rospy = types.ModuleType("rospy")
    rospy.Time = types.SimpleNamespace(now=lambda: 123)
    rospy.logwarn_throttle = lambda *_args: None
    rospy.loginfo_throttle = lambda *_args: None
    rospy.loginfo = lambda *_args: None
    sys.modules["rospy"] = rospy

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.PoseStamped = FakePoseStamped
    geometry_msgs.msg = geometry_msgs_msg
    sys.modules["geometry_msgs"] = geometry_msgs
    sys.modules["geometry_msgs.msg"] = geometry_msgs_msg

    piper_msgs = types.ModuleType("piper_msgs")
    piper_msgs_msg = types.ModuleType("piper_msgs.msg")
    piper_msgs_msg.PiperEulerPose = object
    piper_msgs_msg.PosCmd = FakePosCmd
    piper_msgs.msg = piper_msgs_msg
    sys.modules["piper_msgs"] = piper_msgs
    sys.modules["piper_msgs.msg"] = piper_msgs_msg

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.JointState = FakeJointState
    sensor_msgs.msg = sensor_msgs_msg
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Bool = object
    std_msgs_msg.Float64 = object
    std_msgs_msg.Float64MultiArray = object
    std_msgs_msg.String = object
    std_msgs.msg = std_msgs_msg
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg

    transformations = types.ModuleType("tf.transformations")
    transformations.quaternion_matrix = lambda _q: [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]

    def euler_from_matrix(matrix, axes="sxyz"):
        if axes != "sxyz":
            raise ValueError(axes)
        pitch = math.asin(max(-1.0, min(1.0, -matrix[2][0])))
        roll = math.atan2(matrix[2][1], matrix[2][2])
        yaw = math.atan2(matrix[1][0], matrix[0][0])
        return roll, pitch, yaw

    def euler_matrix(roll, pitch, yaw, axes="sxyz"):
        if axes != "sxyz":
            raise ValueError(axes)
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return [
            [cy * cp, cy * sp * sr - sy * cr,
             cy * sp * cr + sy * sr, 0.0],
            [sy * cp, sy * sp * sr + cy * cr,
             sy * sp * cr - cy * sr, 0.0],
            [-sp, cp * sr, cp * cr, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    transformations.euler_from_matrix = euler_from_matrix
    transformations.euler_matrix = euler_matrix
    tf = types.ModuleType("tf")
    tf.transformations = transformations
    sys.modules["tf"] = tf
    sys.modules["tf.transformations"] = transformations

    piper_sdk = types.ModuleType("piper_sdk")
    piper_sdk.C_PiperForwardKinematics = object
    piper_param_package = types.ModuleType("piper_sdk.piper_param")
    piper_param_manager = types.ModuleType(
        "piper_sdk.piper_param.piper_param_manager"
    )
    piper_param_manager.C_PiperParamManager = object
    piper_param_package.piper_param_manager = piper_param_manager
    piper_sdk.piper_param = piper_param_package
    sys.modules["piper_sdk"] = piper_sdk
    sys.modules["piper_sdk.piper_param"] = piper_param_package
    sys.modules[
        "piper_sdk.piper_param.piper_param_manager"
    ] = piper_param_manager

    script = (
        pathlib.Path(__file__).parents[1]
        / "piper"
        / "scripts"
        / "vla_piper_controller.py"
    )
    spec = importlib.util.spec_from_file_location("vla_piper_controller", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class VlaPiperControllerGripperTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_controller_module()

    def make_controller(self, params=None, fk=None):
        controller = self.module.VlaPiperController.__new__(
            self.module.VlaPiperController
        )
        controller.action_type = "joint_absolute"
        controller.z_min = 0.03
        controller.z_enabled = True
        controller.dry_run = False
        controller.gripper_action_type = "physical_absolute"
        controller.gripper_closed_width_m = 0.0
        controller.gripper_open_width_m = 0.10
        controller.gripper_saturation_margin_m = 0.01
        controller.joint_saturation_margin_rad = 0.05
        controller.hold_requested = False
        controller.received_action_type = "joint_absolute"
        controller.fk = fk or FakeFk()
        controller.params = params or FakeParams()
        controller.joint_command_pub = FakePublisher()
        controller.target_joint_pub = FakePublisher()
        controller.safety_state_pub = FakePublisher()
        controller.safety_reason_pub = FakePublisher()
        controller.target_z_pub = FakePublisher()
        controller.ee_command_pub = FakePublisher()
        controller.target_ee_pub = FakePublisher()
        return controller

    def send(self, raw_gripper):
        controller = self.make_controller()
        action = [0.0, 1.0, -0.8, 0.0, 1.0, 0.0, raw_gripper]
        controller._action_callback(types.SimpleNamespace(data=action))
        return controller

    def assert_approved_as(self, raw_gripper, expected):
        controller = self.send(raw_gripper)
        self.assertEqual(controller.safety_state_pub.messages, ["OK"])
        self.assertEqual(controller.safety_reason_pub.messages, [""])
        self.assertEqual(len(controller.joint_command_pub.messages), 1)
        self.assertAlmostEqual(
            controller.joint_command_pub.messages[0].position[6], expected
        )
        self.assertEqual(
            controller.joint_command_pub.messages[0].position[:6],
            [0.0, 1.0, -0.8, 0.0, 1.0, 0.0],
        )

    def assert_held(self, raw_gripper):
        controller = self.send(raw_gripper)
        self.assertEqual(controller.safety_state_pub.messages, ["HOLD"])
        self.assertTrue(controller.safety_reason_pub.messages[0])
        self.assertEqual(controller.joint_command_pub.messages, [])

    def test_physical_values_inside_range_are_unchanged(self):
        for value in (0.000, 0.040, 0.100):
            with self.subTest(raw_gripper=value):
                self.assert_approved_as(value, value)

    def test_small_extrapolations_are_saturated(self):
        self.assert_approved_as(-0.003, 0.000)
        self.assert_approved_as(0.105, 0.100)

    def test_saturation_margin_boundaries_are_inclusive(self):
        self.assert_approved_as(-0.010, 0.000)
        self.assert_approved_as(0.110, 0.100)

    def test_values_outside_margin_are_held(self):
        self.assert_held(-0.011)
        self.assert_held(0.111)

    def test_nonfinite_values_are_held(self):
        self.assert_held(math.nan)
        self.assert_held(math.inf)

    def test_dry_run_reports_applied_gripper_without_driver_publish(self):
        controller = self.make_controller()
        controller.dry_run = True
        action = [0.0, 1.0, -0.8, 0.0, 1.0, 0.0, -0.003]

        controller._action_callback(types.SimpleNamespace(data=action))

        self.assertEqual(controller.safety_state_pub.messages, ["DRY_RUN"])
        self.assertEqual(controller.safety_reason_pub.messages, [""])
        self.assertEqual(controller.joint_command_pub.messages, [])
        self.assertEqual(len(controller.target_joint_pub.messages), 1)
        self.assertAlmostEqual(
            controller.target_joint_pub.messages[0].position[6], 0.0
        )

    def send_joint5(self, joint5, fk=None):
        controller = self.make_controller(params=FakeJoint5Params(), fk=fk)
        action = [0.0, 1.0, -0.8, 0.0, joint5, 0.0, 0.04]
        controller._action_callback(types.SimpleNamespace(data=action))
        return controller

    def assert_joint5_approved_as(self, raw_joint5, expected_joint5):
        controller = self.send_joint5(raw_joint5)
        self.assertEqual(controller.safety_state_pub.messages, ["OK"])
        self.assertEqual(controller.safety_reason_pub.messages, [""])
        self.assertEqual(len(controller.target_joint_pub.messages), 1)
        self.assertAlmostEqual(
            controller.target_joint_pub.messages[0].position[4], expected_joint5
        )
        self.assertAlmostEqual(
            controller.joint_command_pub.messages[0].position[4], expected_joint5
        )

    def assert_joint5_held(self, raw_joint5):
        controller = self.send_joint5(raw_joint5)
        self.assertEqual(controller.safety_state_pub.messages, ["HOLD"])
        self.assertTrue(controller.safety_reason_pub.messages[0])
        self.assertEqual(controller.target_joint_pub.messages, [])
        self.assertEqual(controller.joint_command_pub.messages, [])

    def test_joint_limit_inside_range_is_unchanged(self):
        self.assert_joint5_approved_as(1.21, 1.21)

    def test_joint_limit_upper_small_extrapolations_are_saturated(self):
        self.assert_joint5_approved_as(1.23, 1.22)   # +0.01 rad
        self.assert_joint5_approved_as(1.269, 1.22)  # +0.049 rad

    def test_joint_limit_upper_outside_margin_is_held(self):
        self.assert_joint5_held(1.271)  # +0.051 rad

    def test_joint_limit_lower_small_extrapolations_are_saturated(self):
        self.assert_joint5_approved_as(-1.23, -1.22)   # -0.01 rad
        self.assert_joint5_approved_as(-1.269, -1.22)  # -0.049 rad

    def test_joint_limit_lower_outside_margin_is_held(self):
        self.assert_joint5_held(-1.271)  # -0.051 rad

    def test_joint_nonfinite_values_are_held(self):
        self.assert_joint5_held(math.nan)
        self.assert_joint5_held(math.inf)

    def test_joint_fk_and_z_gate_use_saturated_target(self):
        fk = FakeFkChecksAppliedJoint()
        controller = self.send_joint5(1.23, fk=fk)

        self.assertEqual(controller.safety_state_pub.messages, ["OK"])
        self.assertEqual(controller.safety_reason_pub.messages, [""])
        self.assertEqual(fk.calls[0][4], 1.22)
        self.assertAlmostEqual(controller.target_z_pub.messages[0], 0.20)

    def make_pbl8_controller(self):
        controller = self.make_controller()
        controller.action_type = "ee_delta_cumulative_reference"
        controller.received_action_type = "ee_delta_cumulative_reference"
        controller.gripper_action_type = "pbl8_normalized_absolute"
        controller.pbl8_max_translation_delta_m = 0.002
        controller.pbl8_max_rotation_delta_rad = 0.01
        controller.pbl8_reference_mode = "cumulative"
        controller.pbl8_feedback_timeout_sec = 0.5
        controller.current_ee_pose = FakePoseStamped()
        controller.current_ee_euler = types.SimpleNamespace(
            roll=0.0, pitch=0.0, yaw=0.0
        )
        controller._current_ee_euler_received_monotonic = time.monotonic()
        controller._pbl8_reference_position = None
        controller._pbl8_reference_rotation = None
        controller._pbl8_command_euler = None
        return controller

    def make_actual_feedback_pbl8_controller(self):
        controller = self.make_pbl8_controller()
        controller.pbl8_reference_mode = "actual_feedback"
        controller.current_ee_euler = types.SimpleNamespace(
            x=0.4, y=0.0, z=0.2,
            roll=0.0, pitch=0.0, yaw=0.0,
        )
        controller._current_ee_euler_received_monotonic = time.monotonic()
        return controller

    def test_euler_unwrap_positive_pi_reference(self):
        epsilon = 1e-8
        result = self.module.VlaPiperController._unwrap_euler_near_reference(
            [-math.pi + epsilon, 0.0, 0.0],
            [math.pi, 0.0, 0.0],
        )
        self.assertAlmostEqual(result[0], math.pi + epsilon)

    def test_euler_unwrap_negative_pi_reference(self):
        epsilon = 1e-8
        result = self.module.VlaPiperController._unwrap_euler_near_reference(
            [math.pi - epsilon, 0.0, 0.0],
            [-math.pi, 0.0, 0.0],
        )
        self.assertAlmostEqual(result[0], -math.pi - epsilon)

    def test_euler_unwrap_preserves_small_changes_on_all_axes(self):
        raw = [0.11, -0.22, 0.33]
        result = self.module.VlaPiperController._unwrap_euler_near_reference(
            raw, [0.10, -0.20, 0.30]
        )
        self.assertEqual(result, raw)

    def test_pbl8_initial_euler_reference_comes_from_feedback(self):
        controller = self.make_pbl8_controller()
        controller.current_ee_euler = types.SimpleNamespace(
            roll=math.pi, pitch=-0.0027, yaw=math.pi
        )

        controller._initialize_pbl8_reference()

        self.assertEqual(
            controller._pbl8_command_euler,
            [math.pi, -0.0027, math.pi],
        )

    def test_pbl8_target_uses_feedback_then_previous_command_for_unwrap(self):
        controller = self.make_pbl8_controller()
        controller.current_ee_euler = types.SimpleNamespace(
            roll=math.pi, pitch=0.0, yaw=math.pi
        )
        raw_rolls = iter([-math.pi + 1e-8, -math.pi + 2e-8])
        original_euler_from_matrix = self.module.euler_from_matrix
        self.module.euler_from_matrix = lambda _matrix, axes="sxyz": (
            next(raw_rolls), 0.0, math.pi
        )
        action = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0]
        try:
            controller._action_callback(types.SimpleNamespace(data=action))
            controller._action_callback(types.SimpleNamespace(data=action))
        finally:
            self.module.euler_from_matrix = original_euler_from_matrix

        first = controller.target_ee_pub.messages[0]
        second = controller.target_ee_pub.messages[1]
        self.assertAlmostEqual(first.roll, math.pi + 1e-8)
        self.assertAlmostEqual(second.roll, math.pi + 2e-8)
        self.assertAlmostEqual(controller._pbl8_command_euler[0], second.roll)
        self.assertAlmostEqual(first.x, 0.401)
        self.assertAlmostEqual(second.x, 0.402)

    def test_pbl8_reference_is_cumulative_and_rotation_is_world_frame(self):
        controller = self.make_pbl8_controller()
        action = [0.001, 0.0, 0.0, 0.0, 0.0, 0.005, -1.0]

        controller._action_callback(types.SimpleNamespace(data=action))
        controller._action_callback(types.SimpleNamespace(data=action))

        self.assertEqual(len(controller.target_ee_pub.messages), 2)
        self.assertAlmostEqual(controller.target_ee_pub.messages[0].x, 0.401)
        self.assertAlmostEqual(controller.target_ee_pub.messages[1].x, 0.402)
        self.assertAlmostEqual(controller.target_ee_pub.messages[0].yaw, 0.005)
        self.assertAlmostEqual(controller.target_ee_pub.messages[1].yaw, 0.010)
        self.assertAlmostEqual(controller.target_ee_pub.messages[0].gripper, 0.10)
        self.assertEqual(controller.safety_state_pub.messages, ["OK", "OK"])
        self.assertEqual(controller.safety_reason_pub.messages, ["", ""])

    def test_pbl8_reference_accumulates_only_runtime_approved_steps(self):
        controller = self.make_pbl8_controller()
        controller.pbl8_max_translation_delta_m = 0.005
        approved = [0.004, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0]

        # Runtime rejects the intervening 6 mm action and never publishes it on
        # /vla/approved_action, so the controller sees only these two 4 mm steps.
        controller._action_callback(types.SimpleNamespace(data=approved))
        controller._action_callback(types.SimpleNamespace(data=approved))

        self.assertEqual(len(controller.target_ee_pub.messages), 2)
        self.assertAlmostEqual(controller.target_ee_pub.messages[0].x, 0.404)
        self.assertAlmostEqual(controller.target_ee_pub.messages[1].x, 0.408)
        self.assertAlmostEqual(controller._pbl8_reference_position[0], 0.408)

    def test_canonical_pbl8_action_type_uses_same_cumulative_handler(self):
        controller = self.make_pbl8_controller()
        controller.action_type = "ee_delta_physical_world_per_control_step"
        controller.received_action_type = controller.action_type

        controller._action_callback(types.SimpleNamespace(
            data=[0.001, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0]
        ))

        self.assertAlmostEqual(controller.target_ee_pub.messages[0].x, 0.401)
        self.assertEqual(controller.safety_state_pub.messages, ["OK"])
        self.assertEqual(controller.safety_reason_pub.messages, [""])

    def test_pbl8_actual_feedback_translation_uses_latest_pose(self):
        controller = self.make_actual_feedback_pbl8_controller()
        action = [0.0, 0.0, 0.002, 0.0, 0.0, 0.0, -1.0]

        controller._action_callback(types.SimpleNamespace(data=action))

        target = controller.target_ee_pub.messages[0]
        self.assertAlmostEqual(target.x, 0.4)
        self.assertAlmostEqual(target.y, 0.0)
        self.assertAlmostEqual(target.z, 0.202)
        self.assertEqual(controller.safety_state_pub.messages, ["OK"])

    def test_pbl8_actual_feedback_does_not_accumulate_without_motion(self):
        controller = self.make_actual_feedback_pbl8_controller()
        action = [0.0, 0.0, 0.002, 0.0, 0.0, 0.0, -1.0]

        controller._action_callback(types.SimpleNamespace(data=action))
        controller._action_callback(types.SimpleNamespace(data=action))

        self.assertAlmostEqual(controller.target_ee_pub.messages[0].z, 0.202)
        self.assertAlmostEqual(controller.target_ee_pub.messages[1].z, 0.202)

    def test_pbl8_actual_feedback_uses_partial_physical_following(self):
        controller = self.make_actual_feedback_pbl8_controller()
        action = [0.0, 0.0, 0.002, 0.0, 0.0, 0.0, -1.0]

        controller._action_callback(types.SimpleNamespace(data=action))
        controller.current_ee_euler.z = 0.2005
        controller._current_ee_euler_received_monotonic = time.monotonic()
        controller._action_callback(types.SimpleNamespace(data=action))

        self.assertAlmostEqual(controller.target_ee_pub.messages[0].z, 0.202)
        self.assertAlmostEqual(controller.target_ee_pub.messages[1].z, 0.2025)

    def test_pbl8_actual_feedback_rotation_uses_latest_orientation(self):
        controller = self.make_actual_feedback_pbl8_controller()
        controller.current_ee_euler.yaw = 0.2
        action = [0.0, 0.0, 0.0, 0.0, 0.0, 0.005, -1.0]

        controller._action_callback(types.SimpleNamespace(data=action))

        self.assertAlmostEqual(controller.target_ee_pub.messages[0].yaw, 0.205)

    def test_pbl8_actual_feedback_unwraps_against_latest_euler(self):
        controller = self.make_actual_feedback_pbl8_controller()
        controller.current_ee_euler.roll = math.pi
        controller.current_ee_euler.yaw = math.pi
        epsilon = 1e-8
        original_euler_from_matrix = self.module.euler_from_matrix
        self.module.euler_from_matrix = lambda _matrix, axes="sxyz": (
            -math.pi + epsilon, 0.0, math.pi
        )
        try:
            controller._action_callback(types.SimpleNamespace(
                data=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0]
            ))
        finally:
            self.module.euler_from_matrix = original_euler_from_matrix

        self.assertAlmostEqual(
            controller.target_ee_pub.messages[0].roll,
            math.pi + epsilon,
        )

    def test_pbl8_actual_feedback_missing_holds(self):
        controller = self.make_actual_feedback_pbl8_controller()
        controller.current_ee_euler = None
        controller._current_ee_euler_received_monotonic = None

        controller._action_callback(types.SimpleNamespace(
            data=[0.0, 0.0, 0.001, 0.0, 0.0, 0.0, -1.0]
        ))

        self.assertEqual(controller.safety_state_pub.messages, ["HOLD"])
        self.assertIn(
            "pbl8_ee_feedback_stale",
            controller.safety_reason_pub.messages[0],
        )
        self.assertEqual(controller.target_ee_pub.messages, [])

    def test_pbl8_actual_feedback_stale_holds(self):
        controller = self.make_actual_feedback_pbl8_controller()
        controller.pbl8_feedback_timeout_sec = 0.1
        controller._current_ee_euler_received_monotonic = time.monotonic() - 1.0

        controller._action_callback(types.SimpleNamespace(
            data=[0.0, 0.0, 0.001, 0.0, 0.0, 0.0, -1.0]
        ))

        self.assertEqual(controller.safety_state_pub.messages, ["HOLD"])
        self.assertIn(
            "pbl8_ee_feedback_stale",
            controller.safety_reason_pub.messages[0],
        )
        self.assertEqual(controller.target_ee_pub.messages, [])

    def test_pbl8_gripper_zero_and_positive_are_closed(self):
        controller = self.make_pbl8_controller()
        self.assertAlmostEqual(controller._gripper_action_to_width_m(0.0), 0.0)
        self.assertAlmostEqual(controller._gripper_action_to_width_m(0.5), 0.0)
        self.assertAlmostEqual(controller._gripper_action_to_width_m(-0.48), 0.048)

    def test_pbl8_gripper_small_extrapolation_is_saturated(self):
        controller = self.make_pbl8_controller()
        self.assertAlmostEqual(
            controller._gripper_action_to_width_m(-1.0009), 0.10
        )
        self.assertAlmostEqual(
            controller._gripper_action_to_width_m(1.0009), 0.0
        )
        self.assertAlmostEqual(
            controller._gripper_action_to_width_m(-1.01), 0.10
        )
        self.assertAlmostEqual(
            controller._gripper_action_to_width_m(1.01), 0.0
        )

    def test_pbl8_out_of_range_delta_holds_and_resets_reference(self):
        controller = self.make_pbl8_controller()
        controller._action_callback(types.SimpleNamespace(
            data=[0.0021, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0]
        ))
        self.assertEqual(controller.safety_state_pub.messages, ["HOLD"])
        self.assertIn(
            "pbl8_translation_delta_limit",
            controller.safety_reason_pub.messages[0],
        )
        self.assertEqual(controller.target_ee_pub.messages, [])
        self.assertIsNone(controller._pbl8_reference_position)
        self.assertIsNone(controller._pbl8_reference_rotation)
        self.assertIsNone(controller._pbl8_command_euler)

    def test_pbl8_out_of_range_gripper_holds(self):
        controller = self.make_pbl8_controller()
        controller._action_callback(types.SimpleNamespace(
            data=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.011]
        ))
        self.assertEqual(controller.safety_state_pub.messages, ["HOLD"])
        self.assertIn(
            "PBL8 gripper command outside",
            controller.safety_reason_pub.messages[0],
        )
        self.assertEqual(controller.target_ee_pub.messages, [])




# Restore fake import modules between suites during unittest discovery.
from _mock_isolation import isolate_imports
setUpModule, tearDownModule = isolate_imports()

if __name__ == "__main__":
    unittest.main()
