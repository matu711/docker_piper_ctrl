#!/usr/bin/env python3
"""Normal drivers with every ROS endpoint and SDK arm replaced by mocks."""
import importlib
import math
import tempfile
import threading
from types import SimpleNamespace
from pathlib import Path
import sys
import unittest
from unittest.mock import MagicMock, patch

from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "piper/scripts"))


class LeaderFollowerTests(unittest.TestCase):
    def driver(self, module_name, class_name, port):
        module = importlib.import_module(module_name)
        ros = MagicMock()
        ros.get_param.side_effect = lambda name, default=None: {
            "~can_port": port, "~auto_enable": False,
        }.get(name, default)
        arm = MagicMock()
        arm.enable_arm.return_value = True
        arm.init.return_value.GetArmStatus.return_value.arm_status.ctrl_mode = 1
        arm.get_joint_states.return_value = ((0., .5, -.5, 0., .5, 0.), 10., 30.)
        arm.get_gripper_states.return_value = ((.04, 0.), 10., 30.)
        with patch.object(module, "rospy", ros), \
             patch.object(module, "Piper", return_value=arm) as factory, \
             patch.object(module.time, "sleep"):
            node = getattr(module, class_name)()
        factory.assert_called_once_with(port)
        # Callbacks continue using the mocked ROS module after construction.
        patcher = patch.object(module, "rospy", ros)
        patcher.start()
        self.addCleanup(patcher.stop)
        return node, arm

    @staticmethod
    def message(values):
        message = JointState()
        message.position = values
        return message

    def test_single_leader_publishes_measured_joints_and_gripper(self):
        leader, arm = self.driver("piper_ctrl_leader_node_new", "PiperLeaderNode", "can1")
        leader.Publish_JointPos(None)
        message = leader.joint_pose_pub.publish.call_args[0][0]
        self.assertEqual(message.position, [0., .5, -.5, 0., .5, 0., .04])
        arm.move_j.assert_not_called()

    def test_single_follower_has_no_fixed_joint_limits(self):
        node, arm = self.driver("piper_ctrl_follower_node", "PiperFollowerNode", "can0")
        # Following must not even consult the fixed pose limits.
        node.params = MagicMock()
        node.params.GetJointLimitParam.side_effect = AssertionError('pose limits used for following')
        node.joint_callback(self.message([0., .5, -.5, 0., .5, 3., .2]))
        arm.move_j.assert_called_once()
        sent, speed = arm.move_j.call_args[0]
        for expected, actual in zip([0., .5, -.5, 0., .5, 3.], sent):
            self.assertAlmostEqual(actual, expected, delta=1e-5)
        self.assertEqual(speed, 40)
        arm.move_gripper.assert_called_once_with(.1, 1)
        applied = node.applied_cmd_pub.publish.call_args[0][0]
        self.assertEqual(applied.position, sent + [.1])

    def test_follower_hold_blocks_motion_but_no_pose_or_outside_pose_does_not(self):
        node, arm = self.driver("piper_ctrl_follower_node", "PiperFollowerNode", "can0")
        command = self.message([0.] * 7)
        node.joint_callback(command)
        self.assertEqual(arm.move_j.call_count, 1)
        node.latest_pose = self.message([0., 0., -.1, 0., 0., math.pi])
        node.joint_callback(command)
        self.assertEqual(arm.move_j.call_count, 2)
        arm.move_j.reset_mock()
        node.leader_control_blocked = True
        node.joint_callback(command)
        arm.move_j.assert_not_called()

    def test_shared_follower_instances_keep_ports_and_commands_independent(self):
        self.driver("piper_ctrl_leader_node_new", "PiperLeaderNode", "can0")
        self.driver("piper_ctrl_leader_node_new", "PiperLeaderNode", "can2")
        right, right_arm = self.driver("piper_ctrl_follower_node", "PiperFollowerNode", "can1")
        left, left_arm = self.driver("piper_ctrl_follower_node", "PiperFollowerNode", "can3")
        pose = self.message([0., 0., .3, 0., 0., math.pi])
        right.latest_pose = left.latest_pose = pose
        right.joint_callback(self.message([0., .5, -.5, 0., .5, 0., .04]))
        left_arm.move_j.assert_not_called()
        left.joint_callback(self.message([.1, .6, -.6, .1, .6, .1, .03]))
        for arm, expected in ((right_arm, [0., .5, -.5, 0., .5, 0.]),
                              (left_arm, [.1, .6, -.6, .1, .6, .1])):
            arm.move_j.assert_called_once()
            for actual, target in zip(arm.move_j.call_args[0][0], expected):
                self.assertAlmostEqual(actual, target, delta=1e-5)

    def test_saved_pose_validator_checks_all_joint_boundaries(self):
        node, _ = self.driver("piper_ctrl_follower_node", "PiperFollowerNode", "can0")
        for index in range(6):
            lower, upper = node.params.GetJointLimitParam("j%d" % (index + 1))
            if index == 1 and lower == 0: lower -= .05
            if index == 2 and upper == 0: upper += .05
            for value, accepted in ((lower, True), (upper, True), (lower-1e-7, False), (upper+1e-7, False)):
                joints = [0.] * 6
                joints[index] = value
                payload = {"joint_positions": joints, "gripper": .01}
                with self.subTest(joint=index+1, value=value):
                    if accepted:
                        self.assertEqual(node._validate_initial_pose(payload), (joints,.01))
                    else:
                        with self.assertRaises(ValueError): node._validate_initial_pose(payload)

    def test_zero_allowance_only_expands_sdk_boundaries_at_zero(self):
        node, _ = self.driver("piper_ctrl_follower_node", "PiperFollowerNode", "can0")
        node.params = MagicMock()
        limits = [(-3., 3.), (0., 3.), (-3., 0.), (-3., 3.), (-3., 3.), (-3., 3.)]
        node.params.GetJointLimitParam.side_effect = lambda key: limits[int(key[1:])-1]
        target = {"joint_positions": [0., -.05, .05, 0., 0., 0.], "gripper": .01}
        self.assertEqual(node._validate_initial_pose(target)[0], target["joint_positions"])
        limits[1], limits[2] = (.01, 3.), (-3., -.01)
        with self.assertRaises(ValueError):
            node._validate_initial_pose(target)

    def test_saved_pose_dispatch_matches_shared_sdk_rounding(self):
        from piper_command_validation import prepare_sdk_command
        node, sdk = self.driver("piper_ctrl_follower_node", "PiperFollowerNode", "can0")
        for target in ([0.,-.036408,.034662,0.,.319011,0.], [0.,.5,-.5,0.,.5,0.]):
            expected, grip = prepare_sdk_command(target,.01,node._validate_initial_pose)
            sdk.move_j.reset_mock()
            node._dispatch_joint_command(target,.01,40)
            sdk.move_j.assert_called_once_with(expected,40)
            sdk.move_gripper.assert_called_with(grip,1)

    def test_bad_commands_stop_and_latch_without_sending_motion(self):
        commands = [[0.] * size for size in (0, 6, 8)]
        for index in range(7):
            for bad in (float('nan'), float('inf'), -float('inf')):
                values = [0.] * 7
                values[index] = bad
                commands.append(values)
        # Old mechanical-range excursions are allowed; SDK integer overflow is not.
        commands += [[0.] * 6 + ['bad'], [0.] * 5 + [1e308, .01]]
        for values in commands:
            with self.subTest(values=values):
                node, arm = self.driver("piper_ctrl_follower_node", "PiperFollowerNode", "can0")
                node.joint_callback(self.message(values))
                self.assertTrue(node.command_fault)
                self.assertTrue(node.leader_control_blocked)
                arm.init.return_value.EmergencyStop.assert_called_once_with(1)
                node.joint_callback(self.message([0.] * 7))
                self.assertFalse(node.resume_leader_control(None).success)
                self.assertFalse(node.set_leader_control_block(SimpleNamespace(data=False)).success)
                self.assertFalse(node.go_initial_pose(None).success)
                arm.move_j.assert_not_called()
                arm.move_gripper.assert_not_called()
                node.applied_cmd_pub.publish.assert_not_called()
                arm.init.return_value.EmergencyStop.assert_called_once_with(1)

    def test_following_clips_zero_boundary_before_rounding(self):
        node, arm = self.driver("piper_ctrl_follower_node", "PiperFollowerNode", "can0")
        # A negative J2 must reach zero before SDK 0.001 degree rounding.
        node.params = MagicMock()
        node.params.GetJointLimitParam.side_effect = lambda key: (0., 3.) if key == 'j2' else (-3., 3.)
        node.joint_callback(self.message([0., -.05, 0., 0., 0., 0., .01]))
        self.assertEqual(arm.move_j.call_args.args[0][1], 0.)
        self.assertIsNone(node.command_fault)

    def test_follow_callback_records_quantized_commands_without_mutating_leader(self):
        from piper_command_validation import prepare_follow_command
        node, sdk = self.driver('piper_ctrl_follower_node', 'PiperFollowerNode', 'can0')
        node._validate_initial_pose = MagicMock(side_effect=AssertionError('strict pose path used'))
        for j2,j3 in ((-.01,.01),(.01,-.01),(4.,-4.)):
            for grip in (-.005,.05,.105,.01234567):
                values=[3.,j2,j3,1.745001,1.5,3.,grip]
                message=self.message(values.copy())
                joints, gripper=prepare_follow_command(values)
                sdk.move_j.reset_mock()
                node.joint_callback(message)
                sdk.move_j.assert_called_once_with(joints,40)
                sdk.move_gripper.assert_called_with(gripper,1)
                self.assertEqual(list(node.applied_cmd_pub.publish.call_args.args[0].position),joints+[gripper])
                self.assertEqual(list(message.position),values)

    def test_saved_pose_dispatch_still_rejects_j4_limit(self):
        node, sdk=self.driver('piper_ctrl_follower_node','PiperFollowerNode','can0')
        with self.assertRaises(ValueError):
            node._dispatch_joint_command([0.,0.,0.,1.745001,0.,0.],.05,20)
        sdk.move_j.assert_not_called()
        sdk.move_gripper.assert_not_called()

    def test_gripper_clip_preserves_both_bounds(self):
        node, arm = self.driver("piper_ctrl_follower_node", "PiperFollowerNode", "can0")
        for raw, expected in ((-.1, 0.), (.2, .1)):
            node.joint_callback(self.message([0.] * 6 + [raw]))
            arm.move_gripper.assert_called_with(expected, 1)
            self.assertIsNone(node.command_fault)

    def test_sdk_send_failure_stops_and_latches_even_without_error_text(self):
        node, arm = self.driver("piper_ctrl_follower_node", "PiperFollowerNode", "can0")
        arm.move_j.side_effect = RuntimeError()
        node.joint_callback(self.message([0.] * 7))
        self.assertTrue(node.command_fault)
        self.assertTrue(node.leader_control_blocked)
        self.assertFalse(node.resume_leader_control(None).success)
        arm.move_gripper.assert_not_called()
        node.applied_cmd_pub.publish.assert_not_called()
        arm.init.return_value.EmergencyStop.assert_called_once_with(1)

    def test_saved_pose_returns_with_hold_and_can_resume(self):
        node, arm = self.driver("piper_ctrl_follower_node", "PiperFollowerNode", "can0")
        module = sys.modules['piper_ctrl_follower_node']
        module.rospy.is_shutdown.return_value = False
        arm.get_end_pose_euler.return_value = ((0., 0., .3, 0., 0., 0.), 10., 30.)
        with tempfile.TemporaryDirectory() as directory:
            node.initial_pose_file = str(Path(directory) / 'initial.yaml')
            self.assertTrue(node.save_initial_pose(None).success)
            self.assertTrue(node.go_initial_pose(None).success)
            self.assertTrue(node.leader_control_blocked)
            self.assertFalse(node.initial_pose_return_active)
            self.assertEqual(arm.move_j.call_args[0][1], 20)
            self.assertTrue(node.resume_leader_control(None).success)
            self.assertFalse(node.leader_control_blocked)




# Restore fake import modules between suites during unittest discovery.
from _mock_isolation import isolate_imports
setUpModule, tearDownModule = isolate_imports()

if __name__ == "__main__":
    unittest.main()
