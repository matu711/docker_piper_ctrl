#!/usr/bin/env python3
"""Unit tests for go_initial_pose without ROS, CAN, or PiPER hardware."""

import importlib.util
import math
import pathlib
import sys
import types
import unittest


class FakeParams:
    def GetJointLimitParam(self, name):
        return {
            "j1": (-2.0, 2.0), "j2": (-0.5, 2.0),
            "j3": (-2.0, 0.5), "j4": (-2.0, 2.0),
            "j5": (-1.0, 2.0), "j6": (-2.0, 2.0),
        }[name]


def load_module():
    rospy = types.ModuleType("rospy")
    sys.modules["rospy"] = rospy
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.JointState = object
    sensor_msgs.msg = sensor_msgs_msg
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Bool = object
    std_msgs.msg = std_msgs_msg
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg
    script = (
        pathlib.Path(__file__).parents[1]
        / "piper" / "scripts" / "go_initial_pose.py"
    )
    spec = importlib.util.spec_from_file_location("go_initial_pose", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class GoInitialPoseTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_module()

    def test_repository_yaml_is_used(self):
        config, target = self.module.load_initial_pose_config(
            pathlib.Path(__file__).parents[1]
            / "piper" / "config" / "piper_demo_start.yaml"
        )
        self.assertEqual(
            target,
            [0.0, 0.555039961, -0.738515702, 0.0, 1.025707780, 0.0, 0.000409231],
        )
        self.assertEqual(config["move_duration"], 3.0)
        self.assertEqual(config["command_rate"], 30.0)

    def test_smoothstep_is_incremental_and_finishes_exactly(self):
        start = [0.0] * 7
        target = [0.1, 1.0, -0.8, 0.2, 0.9, -0.1, 0.04]
        points = self.module.smoothstep_trajectory(start, target, 100)
        self.assertEqual(len(points), 100)
        self.assertNotEqual(points[0], target)
        self.assertEqual(points[-1], target)
        for index in range(7):
            low, high = sorted((start[index], target[index]))
            self.assertTrue(all(low <= point[index] <= high for point in points))

    def test_joint_and_gripper_limits_are_not_clipped(self):
        safe = [0.0, 1.0, -0.8, 0.0, 1.0, 0.0, 0.0004]
        self.assertEqual(
            self.module.validate_pose_limits(safe, FakeParams()), safe
        )
        for bad in (
            [2.01, 1.0, -0.8, 0.0, 1.0, 0.0, 0.0004],
            [0.0, 1.0, -0.8, 0.0, 1.0, 0.0, 0.101],
            [0.0, 1.0, math.nan, 0.0, 1.0, 0.0, 0.04],
        ):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                self.module.validate_pose_limits(bad, FakeParams())

    def test_driver_route_requires_same_feedback_and_command_node(self):
        state = (
            [("/joint_states_single", ["/piper_ctrl_single_node_1"])],
            [
                ("/joint_states", [
                    "/robot_state_publisher", "/piper_ctrl_single_node_1",
                ]),
            ],
            [],
        )
        self.assertEqual(
            self.module.analyze_driver_topology(state),
            ("/joint_states", "/piper_ctrl_single_node_1", True),
        )

    def test_unrelated_joint_state_subscriber_is_not_a_driver(self):
        state = (
            [],
            [("/joint_states", ["/robot_state_publisher"])],
            [],
        )
        self.assertEqual(
            self.module.analyze_driver_topology(state), (None, None, False)
        )

    def test_partial_driver_evidence_fails_closed(self):
        publisher_only = (
            [("/joint_states_single", ["/starting_driver"])], [], [],
        )
        subscriber_only = (
            [], [("/joint_states", ["/piper_ctrl_single_node_2"])], [],
        )
        self.assertEqual(
            self.module.analyze_driver_topology(publisher_only),
            (None, None, True),
        )
        self.assertEqual(
            self.module.analyze_driver_topology(subscriber_only),
            (None, None, True),
        )

    def test_other_piper_driver_topics_prevent_direct_can_fallback(self):
        state = (
            [("/end_pose_euler", ["/piper_ctrl_follower_node"])],
            [],
            [],
        )
        self.assertEqual(
            self.module.analyze_driver_topology(state), (None, None, True)
        )

    def test_namespaced_driver_node_prevents_direct_can_fallback(self):
        state = (
            [("/cell_a/private_feedback", ["/cell_a/piper_ctrl_single_node_7"])],
            [],
            [],
        )
        self.assertEqual(
            self.module.analyze_driver_topology(state), (None, None, True)
        )

    def test_multiple_driver_routes_are_rejected(self):
        state = (
            [("/joint_states_single", ["/driver"])],
            [
                ("/joint_states", ["/driver"]),
                ("/joint_ctrl_single", ["/driver"]),
            ],
            [],
        )
        with self.assertRaises(RuntimeError):
            self.module.analyze_driver_topology(state)




# Restore fake import modules between suites during unittest discovery.
from _mock_isolation import isolate_imports
setUpModule, tearDownModule = isolate_imports()

if __name__ == "__main__":
    unittest.main()
