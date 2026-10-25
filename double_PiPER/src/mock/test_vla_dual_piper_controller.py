#!/usr/bin/env python3
"""Hardware-free differential tests against the unchanged single-arm safety."""
import ast
from collections import deque
import json
import time
import importlib.util
import math
import os
import re
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

from test_vla_piper_controller import (
    FakePublisher, FakeJointState, FakeFk, FakeFkChecksAppliedJoint,
    FakeParams, FakeJoint5Params, load_controller_module,
)


SCRIPTS = Path(__file__).resolve().parents[1] / "piper" / "scripts"
sys.path.insert(0, str(SCRIPTS))


def load(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


class DualControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.single = load_controller_module()
        sys.modules["vla_piper_controller"] = cls.single
        srv = types.ModuleType("std_srvs.srv")
        srv.SetBool = object
        srv.SetBoolResponse = lambda **kwargs: types.SimpleNamespace(**kwargs)
        srv.Trigger = object
        srv.TriggerResponse = lambda **kwargs: types.SimpleNamespace(**kwargs)
        cls.srv_patch = patch.dict(sys.modules, {"std_srvs": types.ModuleType("std_srvs"),
                                                "std_srvs.srv": srv})
        cls.srv_patch.start()
        cls.module = load("vla_dual_piper_controller")

    @classmethod
    def tearDownClass(cls):
        cls.srv_patch.stop()

    def setUp(self):
        self.publishers = {}
        # Synthetic limits for Mock tests only; never deploy these as defaults.
        self.config = {"~max_delta_left": [.15]*6, "~max_delta_right": [.15]*6}
        self.subscriptions = []
        self.subscription_types = {}
        self.publisher_types = {}

        def publisher(topic, *_args, **_kwargs):
            self.publisher_types[topic] = _args[0]
            value = FakePublisher()
            self.publishers[topic] = value
            return value

        def subscriber(topic, kind, callback, **kwargs):
            self.subscription_types[topic] = kind
            self.subscriptions.append((topic, callback, kwargs))
            return Mock()

        self.patches = [
            patch.object(self.module.rospy, "get_param", lambda key, default=None:
                         self.config.get(key, default), create=True),
            patch.object(self.module.rospy, "init_node", Mock(), create=True),
            patch.object(self.module.rospy, "Publisher", publisher, create=True),
            patch.object(self.module.rospy, "Subscriber", subscriber, create=True),
            patch.object(self.module.rospy, "Service", Mock(), create=True),
            patch.object(self.module.rospy, "Timer", Mock(), create=True),
            patch.object(self.module.rospy, "Duration", lambda x: x, create=True),
            patch.object(self.module, "C_PiperForwardKinematics", lambda **kw: FakeFk()),
            patch.object(self.module, "C_PiperParamManager", FakeParams),
        ]
        for p in self.patches:
            p.start()
            self.addCleanup(p.stop)

    def controller(self, dry=False):
        self.config["~dry_run"] = dry
        node = self.module.VlaDualPiperController()
        self.feed_states(node, self.action())
        node.start(None)
        return node

    @staticmethod
    def action():
        return [0., 1., -.8, 0., 1., 0., .04, .2, .8, -.6, .1, .9, -.1, .06]

    def feed_health(self, node, side):
        from dual_feedback import FRAMES
        count = getattr(node, "_test_feedback_count", 0) + 1
        node._test_feedback_count = count
        data = dict(schema=1, epoch=getattr(node, "_test_epoch", "fixture"), healthy=True,
                    frames={key: dict(count=count, age_s=0., source_stamp=float(count)) for key in FRAMES})
        node.feedback_callback(types.SimpleNamespace(data=json.dumps(data)), side)

    def feed_states(self, node, action):
        for side, offset in (("left", 0), ("right", 7)):
            self.feed_health(node, side)
            sample = FakeJointState()
            sample.name = ["joint%d" % j for j in range(1, 7)] + ["gripper"]
            values = []
            for j, value in enumerate(action[offset:offset+6], 1):
                lower, upper = node.arms[side].params.GetJointLimitParam("j%d" % j)
                values.append(min(upper, max(lower, value)) if math.isfinite(value) else 0.)
            sample.position = values + [.04]
            sample.header.stamp = self.module.rospy.Time.now()
            node.state_callback(sample, side)

    def send(self, node, action=None, feedback=True):
        if feedback:
            self.feed_states(node, self.action() if action is None or len(action) != 14 else action)
        node.action_callback(types.SimpleNamespace(
            data=self.action() if action is None else action,
            layout=types.SimpleNamespace(dim=[types.SimpleNamespace(
                label="controller_session:" + str(node.session_token))])))

    def commands(self, node):
        return [arm.joint_command_pub.messages for arm in node.arms.values()]

    def test_initial_hold_and_topics(self):
        node = self.module.VlaDualPiperController()
        self.send(node)
        self.assertEqual(self.commands(node), [[], []])
        self.assertIn("/followerL/joint_states", self.publishers)
        self.assertIn("/followerR/joint_states", self.publishers)
        self.assertEqual({row[0] for row in self.subscriptions}, {
            "/vla_dual/approved_action", "/vla_dual/hold",
            "/followerL/joint_states_single", "/followerR/joint_states_single",
            "/followerL/feedback_health", "/followerR/feedback_health"})

    def test_left_right_absolute_dispatch_without_state_addition(self):
        node = self.controller()
        self.send(node)
        for side, expected in (("left", self.action()[:7]), ("right", self.action()[7:])):
            message = node.arms[side].joint_command_pub.messages[0]
            self.assertEqual(message.position, expected)
            self.assertEqual(message.name, ["joint%d" % i for i in range(1, 7)] + ["gripper"])

    def test_all_safety_methods_are_inherited_without_overrides(self):
        for name in ("handle_joint_absolute", "_send_joint_target",
                     "_gripper_action_to_width_m",
                     "_require_finite_action", "_hold", "_validate_gripper_config",
                     "_validate_joint_saturation_config"):
            self.assertIs(getattr(self.module.StagedArm, name),
                          getattr(self.single.VlaPiperController, name))

    def test_disabled_joint_ranges_allow_outside_state_but_keep_step_clip(self):
        self.config["~controller_joint_limit_enabled"] = False
        node = self.controller()
        for side in ("left", "right"):
            node.states[side].position[3] = 3.1
            node.arms[side].params.GetJointLimitParam = Mock(side_effect=AssertionError("range queried"))
        target = self.action()
        target[3] = target[10] = 4.
        self.send(node, target, feedback=False)
        for arm in node.arms.values():
            self.assertAlmostEqual(arm.joint_command_pub.messages[-1].position[3], 3.25)
        node = self.controller(dry=True)
        self.send(node, target, feedback=False)
        self.assertEqual(self.commands(node), [[], []])
        self.assertEqual(node.safety_state_pub.messages[-1], "DRY_RUN")

    def test_disabled_joint_ranges_keep_finite_z_gripper_and_state_checks(self):
        self.config["~controller_joint_limit_enabled"] = False
        for failure in ("finite", "z", "gripper", "state"):
            with self.subTest(failure=failure):
                node = self.controller()
                target = self.action()
                target[3] = target[10] = 4.
                if failure == "finite": target[10] = math.nan
                elif failure == "z": node.arms["right"].fk = types.SimpleNamespace(CalFK=lambda _: [[0., 0., 0.]])
                elif failure == "gripper": target[13] = .2
                else: node.state_received["right"] -= 1.
                self.send(node, target, feedback=False)
                self.assertEqual(self.commands(node), [[], []])
                self.assertTrue(node.fault_latched)

    def test_joint_range_switch_requires_boolean_and_launch_defaults_enabled(self):
        self.config["~controller_joint_limit_enabled"] = "false"
        with self.assertRaises(ValueError): self.controller()
        root = ET.parse(SCRIPTS.parent / "launch/start_vla_dual_piper.launch").getroot()
        self.assertEqual(root.find("arg[@name='controller_joint_limit_enabled']").get('default'), 'true')
        param = root.find("node[@name='vla_dual_piper_controller']/param[@name='controller_joint_limit_enabled']")
        self.assertEqual(param.get('type'), 'bool')
        self.assertEqual(param.get('value'), '$(arg controller_joint_limit_enabled)')

    def test_constructor_safety_settings_match_single_defaults_and_overrides(self):
        for settings in ({}, {"~joint_saturation_margin_rad": .1,
                              "~gripper_saturation_margin_m": .02,
                              "~gripper_open_width_m": .08,
                              "~z_min": .15, "~z_enabled": False, "~dry_run": False}):
            self.config = settings
            with patch.object(self.single, "C_PiperForwardKinematics", lambda **kw: FakeFk()), \
                    patch.object(self.single, "C_PiperParamManager", FakeParams):
                reference = self.single.VlaPiperController()
            staged = self.module.StagedArm("left", "/followerL")
            for name in ("action_type", "gripper_action_type", "z_min", "z_enabled",
                         "dry_run", "dh_is_offset", "joint_saturation_margin_rad",
                         "gripper_saturation_margin_m", "gripper_closed_width_m",
                         "gripper_open_width_m"):
                self.assertEqual(getattr(reference, name), getattr(staged, name), name)

    def reference(self, arm):
        node = self.single.VlaPiperController.__new__(self.single.VlaPiperController)
        node.__dict__.update(arm.__dict__)
        for name in ("joint_command_pub", "target_joint_pub", "target_z_pub",
                     "safety_state_pub", "safety_reason_pub"):
            setattr(node, name, FakePublisher())
        node.hold_requested = False
        node.received_action_type = "joint_absolute"
        return node

    def test_joint_boundaries_and_finite_checks_match_single_on_each_arm(self):
        for side, offset in (("left", 0), ("right", 7)):
            for joint in range(6):
                for value in (-3.051, -3.05, -3.01, -3., 3., 3.01, 3.05, 3.051,
                              math.nan, math.inf, -math.inf):
                    with self.subTest(side=side, joint=joint, value=value):
                        dual = self.controller()
                        ref = self.reference(dual.arms[side])
                        action = self.action()
                        action[offset + joint] = value
                        ref._action_callback(types.SimpleNamespace(data=action[offset:offset+7]))
                        self.send(dual, action)
                        actual = dual.arms[side]
                        self.assertEqual(actual.safety_state_pub.messages,
                                         ref.safety_state_pub.messages)
                        self.assertEqual(actual.safety_reason_pub.messages,
                                         ref.safety_reason_pub.messages)
                        self.assertEqual([m.position for m in actual.joint_command_pub.messages],
                                         [m.position for m in ref.joint_command_pub.messages])
                        if not ref.joint_command_pub.messages:
                            self.assertEqual(self.commands(dual), [[], []])
                            self.assertTrue(dual.fault_latched)
                            self.assertEqual(len(dual.fault_pub.messages), 1)

    def test_gripper_boundaries_match_single_on_each_arm(self):
        for side, offset in (("left", 0), ("right", 7)):
            for value in (-.011, -.01, -.003, 0., .04, .1, .105, .11, .111,
                          math.nan, math.inf, -math.inf):
                with self.subTest(side=side, value=value):
                    dual = self.controller()
                    ref = self.reference(dual.arms[side])
                    action = self.action()
                    action[offset + 6] = value
                    ref._action_callback(types.SimpleNamespace(data=action[offset:offset+7]))
                    self.send(dual, action)
                    actual = dual.arms[side]
                    self.assertEqual(actual.safety_state_pub.messages, ref.safety_state_pub.messages)
                    self.assertEqual(actual.safety_reason_pub.messages, ref.safety_reason_pub.messages)
                    self.assertEqual([m.position for m in actual.joint_command_pub.messages],
                                     [m.position for m in ref.joint_command_pub.messages])
                    if not ref.joint_command_pub.messages:
                        self.assertEqual(self.commands(dual), [[], []])

    def test_start_ack_is_processed_and_tokens_are_unique(self):
        node = self.module.VlaDualPiperController()
        self.feed_states(node, self.action())
        node.hold_callback(types.SimpleNamespace(data=False))
        self.assertTrue(node.hold_requested)
        ack = node.start(None)
        self.assertTrue(ack.success)
        self.assertRegex(ack.message, r"^[0-9a-f]{32}$")
        self.assertFalse(node.hold_requested)
        self.assertFalse(node.fault_latched)
        self.send(node)
        self.assertEqual([len(c) for c in self.commands(node)], [0, 0])  # default dry-run
        self.assertEqual([len(a.target_joint_pub.messages) for a in node.arms.values()], [1, 1])
        node.controller_fault("test fault")
        node.hold_callback(types.SimpleNamespace(data=False))
        self.assertTrue(node.hold_requested)
        next_ack = node.start(None)
        self.assertNotEqual(ack.message, next_ack.message)
        self.assertFalse(node.hold_requested)

    def test_initial_pose_lock_blocks_start_and_actions_until_explicit_restart(self):
        node = self.controller()
        old_token = node.session_token
        self.assertTrue(node.initial_pose_lock(types.SimpleNamespace(data=True)).success)
        self.assertFalse(node.initial_pose_lock(types.SimpleNamespace(data=True)).success)
        self.assertFalse(node.start(None).success)
        node.hold_callback(types.SimpleNamespace(data=False))
        self.send(node)
        self.assertEqual(self.commands(node), [[], []])
        self.assertTrue(node.initial_pose_lock(types.SimpleNamespace(data=False)).success)
        self.assertTrue(node.hold_requested)
        self.assertTrue(node.fault_latched)
        self.send(node)
        self.assertEqual(self.commands(node), [[], []])
        self.assertTrue(node.start(None).success)
        self.assertNotEqual(old_token, node.session_token)
        self.send(node)
        self.assertEqual([len(c) for c in self.commands(node)], [1, 1])

    def test_fk_uses_clamped_joints_on_both_arms(self):
        node = self.controller()
        action = self.action()
        for side, offset in (("left", 0), ("right", 7)):
            node.arms[side].params = FakeJoint5Params()
            node.arms[side].fk = FakeFkChecksAppliedJoint()
            action[offset + 4] = 1.23
        self.send(node, action)
        for arm in node.arms.values():
            self.assertEqual(arm.fk.calls[0][4], 1.22)
            self.assertEqual(arm.joint_command_pub.messages[0].position[4], 1.22)

    def test_z_boundary_disabled_gate_and_reject_both(self):
        for side in ("left", "right"):
            for z, enabled, accepted in ((.009, True, False), (.01, True, True),
                                         (.011, True, True), (.009, False, True)):
                with self.subTest(side=side, z=z, enabled=enabled):
                    node = self.controller()
                    node.arms[side].fk = types.SimpleNamespace(CalFK=lambda _: [[0., 0., z*1000]])
                    node.arms[side].z_enabled = enabled
                    self.send(node)
                    self.assertEqual([bool(c) for c in self.commands(node)], [accepted]*2)

    def test_validation_order_joint_then_z_then_gripper(self):
        node = self.controller()
        action = self.action()
        action[0], action[6] = 4., .2
        node.arms["left"].fk = Mock()
        self.send(node, action)
        node.arms["left"].fk.CalFK.assert_not_called()
        self.assertIn("joint1_limit_violation", node.safety_reason_pub.messages[-1])
        node.start(None)
        action[0] = 0.
        node.arms["left"].fk.CalFK.return_value = [[0., 0., 0.]]
        self.send(node, action)
        self.assertIn("target_z_below_min", node.safety_reason_pub.messages[-1])
        node.start(None)
        node.arms["left"].fk.CalFK.return_value = [[0., 0., 200.]]
        self.send(node, action)
        self.assertIn("gripper_outside_saturation_margin", node.safety_reason_pub.messages[-1])

    def test_no_dispatch_occurs_before_right_validation(self):
        node = self.controller()
        original = node.arms["right"].handle_joint_absolute
        def validate(action):
            self.assertEqual(self.commands(node), [[], []])
            original(action)
        node.arms["right"].handle_joint_absolute = validate
        self.send(node)
        self.assertEqual([len(c) for c in self.commands(node)], [1, 1])

    def test_malformed_action_and_action_types_reject_both(self):
        for size in (0, 7, 13, 15, 28):
            node = self.controller()
            self.send(node, [0.]*size)
            self.assertEqual(self.commands(node), [[], []])
        for kind in ("joint_delta", "ee_absolute", "unsupported"):
            self.config["~action_type"] = kind
            with self.assertRaises(ValueError):
                self.module.VlaDualPiperController()
        self.config["~action_type"] = "joint_absolute"
        node = self.controller()
        self.send(node)
        self.assertEqual([len(c) for c in self.commands(node)], [1, 1])

    def test_rejected_step_latches_until_explicit_start(self):
        node = self.controller()
        old_token = node.session_token
        bad = self.action()
        bad[13] = .2
        self.send(node, bad)
        self.assertEqual(self.commands(node), [[], []])
        self.assertEqual(len(node.fault_pub.messages), 1)
        self.assertIn("gripper_outside_saturation_margin", node.fault_pub.messages[0])
        self.assertTrue(node.fault_latched)
        self.assertTrue(all(a.prepared is None for a in node.arms.values()))
        node.hold_callback(types.SimpleNamespace(data=False))
        self.send(node)
        self.assertEqual(self.commands(node), [[], []])
        ack = node.start(None)
        self.assertTrue(ack.success)
        self.assertFalse(node.hold_requested)
        self.assertNotEqual(ack.message, old_token)
        node.action_callback(types.SimpleNamespace(data=self.action(), layout=types.SimpleNamespace(
            dim=[types.SimpleNamespace(label="controller_session:" + old_token)])))
        self.assertEqual(self.commands(node), [[], []])
        self.send(node)
        self.assertEqual([len(c) for c in self.commands(node)], [1, 1])

    def test_hold_stop_resume_and_dry_run(self):
        node = self.controller()
        self.assertTrue(node.stop(None).success)
        self.send(node)
        self.assertEqual(self.commands(node), [[], []])
        node.hold_callback(types.SimpleNamespace(data=False))
        self.send(node)
        self.assertEqual(self.commands(node), [[], []])
        node.start(None)
        self.send(node)
        node.hold_callback(types.SimpleNamespace(data=True))
        self.send(node)
        self.assertEqual([len(c) for c in self.commands(node)], [1, 1])
        dry = self.controller(dry=True)
        self.send(dry)
        self.assertEqual(self.commands(dry), [[], []])
        for arm in dry.arms.values():
            self.assertEqual(arm.safety_state_pub.messages, ["DRY_RUN"])
            self.assertEqual(len(arm.target_joint_pub.messages), 1)

    def test_dispatch_failure_notifies_runtime_and_suppresses_remaining_arm(self):
        node = self.controller()
        node.arms["left"].joint_command_pub.publish = Mock(side_effect=RuntimeError("transport"))
        self.send(node)
        self.assertEqual(self.commands(node), [[], []])
        self.assertTrue(node.hold_requested)
        self.assertIn("dispatch_failed:left:transport", node.fault_pub.messages[-1])
        self.assertTrue(all(a.prepared is None for a in node.arms.values()))

    def test_step_limits_require_explicit_valid_six_joint_parameters(self):
        for side in ("left", "right"):
            for value in (None, [], [.1]*5, [.1]*7, [0.]*6, [-.1]*6,
                          [math.nan]*6, [math.inf]*6, [True]*6):
                with self.subTest(side=side, value=value):
                    self.config["~max_delta_" + side] = value
                    with self.assertRaises(ValueError):
                        self.module.VlaDualPiperController()
            self.config["~max_delta_" + side] = [.15]*6

    def test_step_clip_all_twelve_joints_with_individual_limits_and_logging(self):
        self.config["~max_delta_left"] = [.01*(j+1) for j in range(6)]
        self.config["~max_delta_right"] = [.02*(j+1) for j in range(6)]
        node = self.controller()
        self.feed_states(node, [0.]*14)
        target = [1., -1., 1., -1., 1., -1., .03]*2
        with patch.object(self.module.rospy, "loginfo") as log:
            self.send(node, target, feedback=False)
        for side in ("left", "right"):
            result = node.arms[side].joint_command_pub.messages[-1].position
            for j, maximum in enumerate(node.max_delta[side]):
                self.assertAlmostEqual(result[j], maximum if j % 2 == 0 else -maximum)
            self.assertEqual(result[6], .03)  # gripper is not step clipped
        self.assertEqual(sum("JOINT_CLIP" in c.args[0] for c in log.call_args_list), 2)

    def test_range_clip_happens_before_step_clip_on_both_bounds(self):
        for sign in (-1, 1):
            node = self.controller()
            self.feed_states(node, [sign*2.98]*6+[.04]+[sign*2.98]*6+[.04])
            self.send(node, [sign*3.02]*6+[.04]+[sign*3.02]*6+[.04], feedback=False)
            for arm in node.arms.values():
                self.assertEqual(arm.joint_command_pub.messages[-1].position[:6], [sign*3.]*6)

    def test_step_clip_is_centered_on_measured_nonzero_state(self):
        node = self.controller()
        current = [.4, -.6, .2, .3, -.7, .1]
        self.feed_states(node, current+[.04]+current+[.04])
        target = [v + (1. if j % 2 == 0 else -1.) for j, v in enumerate(current)]
        self.send(node, target+[.09]+target+[.09], feedback=False)
        for arm in node.arms.values():
            result = arm.joint_command_pub.messages[-1].position
            for j in range(6):
                self.assertAlmostEqual(result[j], current[j]+(.15 if j % 2 == 0 else -.15))
            self.assertEqual(result[6], .09)

    def test_state_expiring_during_fk_stops_both_before_dispatch(self):
        clock = [time.monotonic()]
        with patch.object(self.module.time, "monotonic", lambda: clock[0]):
            node = self.controller()
            def fk(_joints):
                clock[0] += .6
                return [[0., 0., 200.]]
            node.arms["right"].fk = types.SimpleNamespace(CalFK=fk)
            self.send(node, feedback=False)
        self.assertEqual(self.commands(node), [[], []])
        self.assertIn("stale", node.fault_pub.messages[-1])

    def test_out_of_bounds_measured_joint_is_not_clamped_into_a_reference(self):
        node = self.controller()
        sample = FakeJointState()
        sample.name = list(node.states["right"].name)
        sample.header.stamp = self.module.rospy.Time.now()
        sample.position = [3.01]+[0.]*5+[.04]
        node.state_callback(sample, "right")
        self.send(node, feedback=False)
        self.assertEqual(self.commands(node), [[], []])
        self.assertIn("measured joint1 outside SDK limits", node.fault_pub.messages[-1])

    def test_large_outlier_cannot_be_hidden_by_step_clip_on_any_joint(self):
        for offset in (0, 7):
            for j in range(6):
                for sign in (-1, 1):
                    node = self.controller()
                    self.feed_states(node, [0.]*14)
                    target = self.action()
                    target[offset+j] = sign*3.051
                    self.send(node, target, feedback=False)
                    self.assertEqual(self.commands(node), [[], []])
                    self.assertIn("limit_violation", node.fault_pub.messages[-1])

    def test_missing_stale_future_and_frozen_source_state_hold_both(self):
        for side in ("left", "right"):
            for failure in ("missing", "local_stale", "source_stale", "future"):
                with self.subTest(side=side, failure=failure):
                    node = self.controller()
                    if failure == "missing": node.states.pop(side)
                    elif failure == "local_stale": node.state_received[side] -= 1.
                    elif failure == "source_stale": node.states[side].header.stamp -= 1.
                    else: node.states[side].header.stamp += 1.
                    self.send(node, feedback=False)
                    self.assertEqual(self.commands(node), [[], []])
                    self.assertTrue(node.hold_requested)
                    self.assertTrue(node.fault_latched)
                    self.assertEqual(len(node.fault_pub.messages), 1)

    def test_watchdog_faults_without_a_new_action_and_requires_restart(self):
        node = self.controller()
        node.state_received["right"] -= 1.
        node.check_state_freshness(None)
        self.assertTrue(node.fault_latched)
        self.feed_states(node, self.action())
        node.hold_callback(types.SimpleNamespace(data=False))
        self.send(node, feedback=False)
        self.assertEqual(self.commands(node), [[], []])
        self.assertTrue(node.start(None).success)
        self.send(node, feedback=False)
        self.assertEqual([len(c) for c in self.commands(node)], [1, 1])

    def test_fk_and_z_use_step_clipped_target_and_reject_either_arm(self):
        for safe_after_clip in (True, False):
            node = self.controller()
            self.feed_states(node, [0.]*14)
            target = [1.]*6+[.04]+[1.]*6+[.04]
            def fk(joints):
                self.assertTrue(all(abs(j) <= .15 for j in joints))
                return [[0., 0., 200. if safe_after_clip else 0.]]
            node.arms["right"].fk = types.SimpleNamespace(CalFK=fk)
            self.send(node, target, feedback=False)
            self.assertEqual([bool(c) for c in self.commands(node)], [safe_after_clip]*2)

    def test_nonfinite_fk_z_and_nonfinite_action_prevent_both_sends(self):
        for value in (math.nan, math.inf, -math.inf):
            node = self.controller()
            node.arms["right"].fk = types.SimpleNamespace(CalFK=lambda _: [[0., 0., value]])
            self.send(node)
            self.assertEqual(self.commands(node), [[], []])
            node = self.controller()
            node.arms["left"].fk = Mock()
            target = self.action()
            target[12] = value
            self.send(node, target, feedback=False)
            node.arms["left"].fk.CalFK.assert_not_called()
            self.assertEqual(self.commands(node), [[], []])

    def test_dry_run_reports_clipped_target_without_driver_commands(self):
        node = self.controller(dry=True)
        self.feed_states(node, [0.]*14)
        self.send(node, [1.]*6+[.04]+[-1.]*6+[.06], feedback=False)
        self.assertEqual(self.commands(node), [[], []])
        self.assertEqual(node.arms["left"].target_joint_pub.messages[-1].position, [.15]*6+[.04])
        self.assertEqual(node.arms["right"].target_joint_pub.messages[-1].position, [-.15]*6+[.06])
        self.assertEqual(node.safety_state_pub.messages[-1], "DRY_RUN")

    def test_invalid_state_clears_cache_and_named_state_is_reordered(self):
        node = self.controller()
        sample = FakeJointState()
        sample.header.stamp = self.module.rospy.Time.now()
        sample.name = list(reversed(node.states["left"].name))
        sample.position = list(reversed(self.action()[:7]))
        node.state_callback(sample, "left")
        self.assertEqual(node.states["left"].position, self.action()[:7])
        sample.position[0] = math.nan
        node.state_callback(sample, "left")
        self.assertNotIn("left", node.states)
        self.send(node, feedback=False)
        self.assertEqual(self.commands(node), [[], []])
        self.assertTrue(node.hold_requested)
        self.assertTrue(all(a.prepared is None for a in node.arms.values()))

    def test_actual_runtime_tick_stop_and_fault_connect_to_controller(self):
        # Execute the actual Runtime methods and chunk state machine from the
        # read-only sibling source, replacing only ROS/array/model boundaries.
        runtime_dir = os.environ.get("PIPER_VLA_RUNTIME_DIR")
        if not runtime_dir:
            self.skipTest("Set PIPER_VLA_RUNTIME_DIR for optional external Runtime tests")
        runtime_root = Path(runtime_dir).expanduser()
        source = runtime_root / "src/piper_vla_dual_runtime"
        if not source.is_dir():
            self.skipTest("Sibling dual Runtime source unavailable")
        tree = ast.parse((source / "node.py").read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "Runtime")
        methods = {"tick", "_fault", "_controller_fault", "start", "stop", "_publish", "_record", "_controller_start"}
        cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in methods]
        core = ast.parse((source / "core.py").read_text())
        chunk_class = next(n for n in core.body if isinstance(n, ast.ClassDef) and n.name == "Chunks")
        def wire(**kw):
            if "data" in kw:
                kw["layout"] = types.SimpleNamespace(dim=[])
            return types.SimpleNamespace(**kw)
        env = dict(re=re, threading=__import__("threading"), time=time, deque=deque, json=json,
                   Bool=wire, String=wire, Float64MultiArray=wire, MultiArrayDimension=wire,
                   StartInferenceResponse=lambda *a: types.SimpleNamespace(success=a[0], message=a[1]),
                   Trigger=object, TriggerResponse=lambda *a: a,
                   PREFIX="/vla_dual", rospy=types.SimpleNamespace(logwarn=Mock(), loginfo=Mock(), wait_for_service=Mock()))
        exec(compile(ast.Module(body=[chunk_class, cls], type_ignores=[]), str(source), "exec"), env)
        runtime = env["Runtime"]()
        runtime.lock = __import__("threading").RLock()
        runtime.service_lock = __import__("threading").Lock()
        runtime.chunks = env["Chunks"]()
        runtime.chunks.start()
        runtime.recorder = None
        runtime.execution_steps = 0
        runtime.max_execution_steps = 0
        runtime.request = __import__("threading").Event()
        runtime.fault_reason = None
        runtime.inflight_generation = runtime.inflight_since = None
        runtime.cfg = {"runtime": {"inference_timeout_s": 2., "startup_timeout_s": 5.,
                                   "controller_start_timeout_s": 2.},
                       "recording": {"enabled": False}}
        runtime.started = time.monotonic()
        runtime.session_id = 1
        runtime._snapshot = Mock()
        runtime._close_recording = Mock()
        runtime.observations = types.SimpleNamespace(versions={})
        self.config["~dry_run"] = False
        controller = self.module.VlaDualPiperController()
        self.feed_states(controller, self.action())
        runtime.controller_token = None
        runtime.controller_started = None
        runtime.controller_tokens_seen = set()
        runtime.controller_call_lock = __import__("threading").Lock()
        entered, release = __import__("threading").Event(), __import__("threading").Event()
        workers = []
        def start_service():
            entered.set()
            if not release.wait(2):
                raise RuntimeError("test did not release ACK")
            return controller.start(None)
        env["rospy"].ServiceProxy = lambda *a: start_service
        def worker(**kwargs):
            thread = __import__("threading").Thread(**kwargs)
            workers.append(thread)
            return thread
        env["threading"] = types.SimpleNamespace(Thread=worker)
        runtime.pubs = {k: FakePublisher() for k in (
            "hold", "selected_action", "approved_action", "execution_step", "safety_state", "safety_reason",
            "current_prompt")}
        runtime.pubs["hold"].publish = controller.hold_callback
        runtime.pubs["approved_action"].publish = controller.action_callback
        controller.fault_pub.publish = lambda reason: runtime._controller_fault(wire(data=reason))
        class Action(list):
            def tolist(self):
                return list(self)
        def pending():
            runtime.chunks.pending = (1, [Action(self.action()) for _ in range(20)])
        runtime.dry = True
        pending()
        runtime.tick()
        self.assertEqual(self.commands(controller), [[], []])
        self.assertFalse(entered.is_set())
        runtime.stop(None)
        runtime.dry = False
        self.assertTrue(runtime.start(types.SimpleNamespace(prompt="live")).success)
        pending()
        runtime.tick()
        self.assertTrue(entered.wait(2), repr(runtime.fault_reason))
        runtime.tick()
        self.assertEqual(self.commands(controller), [[], []])
        self.assertIsNone(runtime.controller_token)
        release.set()
        workers[-1].join(2)
        self.assertFalse(workers[-1].is_alive())
        self.assertEqual(runtime.controller_token, controller.session_token)
        self.assertFalse(controller.hold_requested)
        for _ in range(19):
            runtime.tick()
        self.assertEqual([len(c) for c in self.commands(controller)], [19, 19], repr(runtime.fault_reason))
        self.assertTrue(runtime.request.is_set())
        runtime.stop(None)
        self.assertTrue(controller.hold_requested)
        self.assertFalse(runtime.chunks.enabled)
        runtime.tick()
        self.assertEqual([len(c) for c in self.commands(controller)], [19, 19], repr(runtime.fault_reason))
        # Actual Runtime start -> ACK -> invalid right Action -> both stop.
        self.assertTrue(runtime.start(types.SimpleNamespace(prompt="restart")).success)
        pending()
        runtime.tick()
        workers[-1].join(2)
        self.assertFalse(workers[-1].is_alive())
        bad = self.action()
        bad[13] = .2
        runtime.chunks.pending = (1, [Action(bad) for _ in range(20)])
        runtime.tick()
        self.assertFalse(runtime.chunks.enabled)
        self.assertTrue(controller.fault_latched)
        self.assertIn("gripper_outside_saturation_margin", runtime.fault_reason)
        self.assertEqual([len(c) for c in self.commands(controller)], [19, 19], repr(runtime.fault_reason))
        controller.hold_callback(types.SimpleNamespace(data=False))
        self.send(controller)
        self.assertEqual([len(c) for c in self.commands(controller)], [19, 19], repr(runtime.fault_reason))
        # A new explicit prompt start, without manual reset/HOLD operations.
        old_token = controller.session_token
        self.assertTrue(runtime.start(types.SimpleNamespace(prompt="new prompt")).success)
        pending()
        runtime.tick()
        workers[-1].join(2)
        self.assertFalse(workers[-1].is_alive())
        self.assertNotEqual(controller.session_token, old_token)
        runtime.tick()
        self.assertEqual([len(c) for c in self.commands(controller)], [20, 20])
        # Actual CAN-progress watchdog -> Controller fault -> Runtime discards
        # its active/pending chunk even when no new Action arrives.
        real_clock = controller.feedback.clock
        controller.feedback.clock = lambda: real_clock()+1.
        controller.check_state_freshness(None)
        self.assertFalse(runtime.chunks.enabled)
        self.assertIsNone(runtime.chunks.pending)
        self.assertFalse(runtime.chunks.active)
        self.assertIn("CAN feedback", runtime.fault_reason)
        runtime.tick()
        self.assertEqual([len(c) for c in self.commands(controller)], [20,20])
        controller.feedback.clock = real_clock
        self.feed_states(controller, self.action())
        self.assertFalse(runtime.chunks.enabled)  # Reception alone cannot resume.
        self.assertTrue(runtime.start(types.SimpleNamespace(prompt="after CAN recovery")).success)
        pending()
        runtime.tick()
        workers[-1].join(2)
        runtime.tick()
        self.assertEqual([len(c) for c in self.commands(controller)], [21,21])
        controller.arms["left"].joint_command_pub.publish = Mock(side_effect=RuntimeError("transport"))
        runtime.tick()
        self.assertFalse(runtime.chunks.enabled)
        self.assertTrue(controller.hold_requested)
        self.assertIn("dispatch_failed:left", runtime.fault_reason)

        # Publication budgets count [left7,right7] packets, including when the
        # Controller is dry. These are Mock limits, never hardware settings.
        controller.arms["left"].joint_command_pub = FakePublisher()
        for controller_dry in (False, True):
            for limit in (1, 10, 20, 21):
                for arm in controller.arms.values():
                    arm.dry_run = controller_dry
                self.feed_states(controller, self.action())
                runtime.max_execution_steps = limit
                before = [len(c) for c in self.commands(controller)]
                targets = [len(a.target_joint_pub.messages) for a in controller.arms.values()]
                self.assertTrue(runtime.start(types.SimpleNamespace(prompt="bounded mock")).success)
                self.assertEqual(runtime.execution_steps, 0)
                pending()
                runtime.tick()
                workers[-1].join(2)
                self.assertFalse(workers[-1].is_alive())
                for _ in range(limit+2):
                    runtime.tick()
                    if runtime.chunks.enabled and runtime.request.is_set():
                        runtime.request.clear()
                        if runtime.chunks.pending is None:
                            runtime.chunks.pending = (2, [Action(self.action()) for _ in range(20)])
                self.assertFalse(runtime.chunks.enabled)
                self.assertEqual(runtime.execution_steps, limit)
                self.assertEqual(runtime.fault_reason, "max_execution_steps_reached")
                self.assertTrue(controller.hold_requested)
                self.assertIsNone(runtime.chunks.pending)
                self.assertFalse(runtime.chunks.active)
                delta = [len(c)-n for c,n in zip(self.commands(controller),before)]
                self.assertEqual(delta, [0,0] if controller_dry else [limit,limit])
                self.assertEqual([len(a.target_joint_pub.messages)-n for a,n in zip(controller.arms.values(),targets)],
                                 [limit,limit])
                runtime.tick()
                self.assertEqual(runtime.execution_steps,limit)

    def test_runtime_topic_types_and_camera_state_config_match(self):
        runtime_dir = os.environ.get("PIPER_VLA_RUNTIME_DIR")
        if not runtime_dir:
            self.skipTest("Set PIPER_VLA_RUNTIME_DIR for optional external Runtime tests")
        runtime_root = Path(runtime_dir).expanduser()
        if not runtime_root.is_dir():
            self.skipTest("Sibling dual Runtime unavailable")
        source = ast.parse((runtime_root / "src/piper_vla_dual_runtime/node.py").read_text())
        prefix = next(n.value.value for n in source.body if isinstance(n, ast.Assign)
                      and any(isinstance(t, ast.Name) and t.id == "PREFIX" for t in n.targets))
        self.assertEqual(prefix, "/vla_dual")
        cls = next(n for n in source.body if isinstance(n, ast.ClassDef) and n.name == "Runtime")
        init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "__init__")
        services = {n.args[0].value: n.args[1].id for n in ast.walk(init)
                    if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                    and n.func.attr == "Service"}
        self.assertEqual(services, {"~start": "StartInference", "~stop": "Trigger"})
        self.assertTrue(any(isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                            and n.func.attr == "init_node"
                            and n.args[0].value == "piper_vla_dual_runtime" for n in ast.walk(init)))
        srv = runtime_root / "ros/piper_vla_runtime_msgs/srv/StartInference.srv"
        self.assertEqual(srv.read_text().splitlines(),
                         ["string prompt", "---", "bool success", "string message"])
        pubs = next(n.value for n in init.body if isinstance(n, ast.Assign)
                    and any(isinstance(t, ast.Attribute) and t.attr == "pubs" for t in n.targets))
        runtime_types = {row.elts[0].value: row.elts[1].id for row in pubs.generators[0].iter.elts}
        self.assertEqual(runtime_types["approved_action"], "Float64MultiArray")
        self.assertEqual(runtime_types["hold"], "Bool")
        node = self.controller()
        self.assertIs(self.subscription_types[prefix + "/approved_action"], self.module.Float64MultiArray)
        self.assertIs(self.subscription_types[prefix + "/hold"], self.module.Bool)
        for side in ("left", "right"):
            self.assertIs(self.publisher_types[prefix + "/" + side + "/target_joint_command"],
                          self.module.JointState)
        self.assertIs(self.publisher_types[prefix + "/controller_fault"], self.module.String)
        self.assertTrue(any(isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                            and n.func.attr == "Subscriber" and len(n.args) > 1
                            and isinstance(n.args[1], ast.Name) and n.args[1].id == "String"
                            and isinstance(n.args[0], ast.BinOp)
                            and isinstance(n.args[0].right, ast.Constant)
                            and n.args[0].right.value == "/controller_fault" for n in ast.walk(init)))
        text = (runtime_root / "configs/runtime.dual.yaml").read_text()
        configured = dict(line.strip().split(": ", 1) for line in text.splitlines()
                          if "_topic: " in line)
        self.assertEqual(configured, {
            "left_topic": "/followerL/joint_states_single", "right_topic": "/followerR/joint_states_single",
            "front_topic": "/camera/front/compressed", "left_wrist_topic": "/camera/left_wrist/compressed",
            "right_wrist_topic": "/camera/right_wrist/compressed"})

    def test_mock_driver_preserves_jointstate_without_sdk(self):
        module = load("vla_dual_mock_driver")
        node = module.MockDriver()
        action = FakeJointState()
        action.position = self.action()[:7]
        node.command(action)
        node.publish_once()
        state = node.publisher.messages[-1]
        self.assertEqual(state.position, action.position)
        self.assertEqual(state.name, ["joint%d" % i for i in range(1, 7)] + ["gripper"])
        tree = ast.parse((SCRIPTS / "vla_dual_mock_driver.py").read_text())
        imports = {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)}
        imports.update(a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names)
        self.assertEqual(imports, {"threading", "json", "time", "rospy", "sensor_msgs.msg", "std_msgs.msg", "dual_feedback"})

    def test_mock_driver_can_stop_keeps_heartbeat_but_not_fresh_state(self):
        module = load("vla_dual_mock_driver")
        node = module.MockDriver()
        node.publish_once()
        first_stamp = node.publisher.messages[-1].header.stamp
        node.feedback_enabled = False
        clock = time.monotonic()+1.
        node.feedback.clock = lambda: clock
        node.publish_once()
        node.publish_once()
        self.assertEqual(len(node.publisher.messages), 1)
        self.assertEqual(node.publisher.messages[-1].header.stamp, first_stamp)
        self.assertFalse(json.loads(node.feedback_pub.messages[-1])["healthy"])
        node.feedback_enabled = True
        node.publish_once()
        self.assertEqual(len(node.publisher.messages), 2)
        self.assertTrue(json.loads(node.feedback_pub.messages[-1])["healthy"])

    def test_combined_state_left_then_right_uses_older_stamp(self):
        node = self.module.VlaDualPiperController()
        self.feed_health(node, "left")
        self.feed_health(node, "right")
        left, right = FakeJointState(), FakeJointState()
        left.name = right.name = ["joint%d" % j for j in range(1, 7)] + ["gripper"]
        left.position, right.position = [1.]*7, [2.]*7
        left.header.stamp, right.header.stamp = 10, 11
        node.state_callback(right, "right")
        self.assertEqual(node.state_pub.messages, [])
        node.state_callback(left, "left")
        msg = node.state_pub.messages[-1]
        self.assertEqual(msg.position, [1.]*7 + [2.]*7)
        self.assertEqual(msg.header.stamp, 10)
        self.assertEqual(msg.name[0], "left_joint1")
        self.assertEqual(msg.name[13], "right_gripper")

    def test_can_stop_with_fresh_ros_state_and_repeated_heartbeat_holds_both(self):
        for failed_side in ("left", "right"):
            with self.subTest(side=failed_side):
                node = self.controller()
                baseline = time.monotonic()
                frozen = node.feedback.arms[failed_side]
                packet = dict(schema=1, epoch=frozen["epoch"], healthy=True,
                              frames={key: dict(count=value["count"], age_s=0., source_stamp=value["source_stamp"])
                                      for key, value in frozen["frames"].items()})
                node.feedback.clock = lambda: baseline + .6
                with patch.object(self.module.time, "monotonic", return_value=baseline+.6):
                    other = "right" if failed_side == "left" else "left"
                    self.feed_health(node, other)
                    # Reproduce the bug: ROS still publishes current timestamps.
                    for side in ("left", "right"):
                        sample = FakeJointState()
                        sample.name = ["joint%d" % j for j in range(1,7)]+["gripper"]
                        sample.position = self.action()[:7] if side == "left" else self.action()[7:]
                        sample.header.stamp = self.module.rospy.Time.now()
                        node.state_callback(sample, side)
                    node.feedback_callback(types.SimpleNamespace(data=json.dumps(packet)), failed_side)
                    node.check_state_freshness(None)
                    self.assertTrue(node.fault_latched)
                    self.assertTrue(node.hold_requested)
                    self.assertIn("CAN feedback", node.fault_pub.messages[-1])
                    self.send(node, feedback=False)
                    self.assertEqual(self.commands(node), [[], []])
                    self.assertFalse(node.start(None).success)

    def test_silent_feedback_stop_watchdog_without_action(self):
        node = self.controller()
        node.feedback.clock = lambda: time.monotonic()+1.
        node.check_state_freshness(None)
        self.assertTrue(node.fault_latched)
        self.assertEqual(self.commands(node), [[], []])

    def test_feedback_recovers_but_requires_explicit_start(self):
        node = self.controller()
        packet = dict(schema=1, epoch="fixture", healthy=False, frames={})
        node.feedback_callback(types.SimpleNamespace(data=json.dumps(packet)), "right")
        self.feed_states(node, self.action())
        node.hold_callback(types.SimpleNamespace(data=False))
        self.send(node, feedback=False)
        self.assertEqual(self.commands(node), [[], []])
        self.assertTrue(node.start(None).success)
        self.send(node, feedback=False)
        self.assertEqual([len(c) for c in self.commands(node)], [1,1])

    def test_driver_restart_and_communication_error_latch(self):
        node = self.controller()
        node._test_epoch = "new-process"
        self.feed_states(node, self.action())
        self.feed_states(node, self.action())
        self.assertTrue(node.fault_latched)
        self.assertEqual(self.commands(node), [[], []])
        self.assertTrue(node.start(None).success)
        from dual_feedback import FRAMES
        node.feedback_callback(types.SimpleNamespace(data=json.dumps(dict(schema=1, epoch="new-process",
            healthy=False, communication_error=True, frames={key: dict(count=100,age_s=0.,source_stamp=100.) for key in FRAMES}))), "left")
        self.assertTrue(node.fault_latched)
        self.send(node, feedback=False)
        self.assertEqual(self.commands(node), [[], []])



class LaunchTests(unittest.TestCase):
    def test_defaults_match_single_and_driver_namespaces_are_separate(self):
        launches = SCRIPTS.parent / "launch"
        single = ET.parse(launches / "start_vla_piper.launch").getroot()
        dual = ET.parse(launches / "start_vla_dual_piper.launch").getroot()
        a = {x.get("name"): x.get("default") for x in single.findall("arg")}
        b = {x.get("name"): x.get("default") for x in dual.findall("arg")}
        for key in ("z_min", "z_enabled", "dry_run", "dh_is_offset", "joint_saturation_margin_rad",
                    "gripper_closed_width_m", "gripper_open_width_m", "gripper_saturation_margin_m",
                    "gripper_max_width_m", "gripper_val_mutiple", "auto_enable"):
            self.assertEqual(a[key], b[key], key)
        self.assertNotEqual(b["left_can_port"], b["right_can_port"])
        for group in dual.findall("group"):
            self.assertIn(group.get("ns"), ("followerL", "followerR"))
            node = group.find("node")
            self.assertEqual(node.get("type"), "piper_ctrl_dual_feedback_node.py")
            self.assertEqual(node.find("remap").attrib,
                             {"from": "joint_ctrl_single", "to": "joint_states"})
        self.assertFalse(any("leader" in n.get("type", "").lower() for n in dual.iter("node")))

    def test_launch_dry_run_enable_and_mock_driver_selection(self):
        root = ET.parse(SCRIPTS.parent / "launch/start_vla_dual_piper.launch").getroot()
        def resolve(value, args):
            if value.startswith("$(eval "):
                return eval(value[7:-1], {"__builtins__": {}}, {"arg": args.__getitem__})
            if value.startswith("$(arg "):
                return args[value[6:-1]]
            return value
        for dry in (True, False):
            for auto in (True, False):
                for mock in (True, False):
                    args = dict(dry_run=dry, auto_enable=auto, mock_driver=mock)
                    for group in root.findall("group"):
                        real, fake = group.findall("node")
                        self.assertEqual(resolve(real.get("unless"), args), mock)
                        self.assertEqual(resolve(fake.get("if"), args), mock)
                        enable = next(p for p in real.findall("param") if p.get("name") == "auto_enable")
                        self.assertEqual(resolve(enable.get("value"), args), auto and not dry)
                    cameras = next(a for a in root.findall("arg") if a.get("name") == "enable_cameras")
                    self.assertEqual(resolve(cameras.get("default"), args), not mock)


class CameraTests(unittest.TestCase):
    def test_three_camera_topics_with_mock_capture_and_encoder(self):
        fake_ros = types.ModuleType("rospy")
        fake_ros.get_param = lambda _, default=None: default
        topics = []
        def publisher(topic, *args, **kwargs):
            topics.append(topic)
            return FakePublisher()
        fake_ros.Publisher = publisher
        fake_ros.logwarn_throttle = Mock()
        camera_base = types.ModuleType("dual_compressed_camera_publisher")
        camera_base.CameraCaptureWorker = Mock()
        frame = Mock()
        camera_base.LatestFrame = lambda: types.SimpleNamespace(get_copy=lambda: (frame, 12, 0))
        camera_base.make_message = Mock(side_effect=lambda f, stamp, name, quality:
                                       (types.SimpleNamespace(frame_id=name, stamp=stamp), 0))
        camera_base.validate_camera_config = Mock()
        camera_base.wait_for_workers = Mock()
        sensor = types.ModuleType("sensor_msgs.msg")
        sensor.CompressedImage = object
        with patch.dict(sys.modules, {"rospy": fake_ros, "cv2": types.SimpleNamespace(CAP_V4L2=200),
                                     "sensor_msgs.msg": sensor,
                                     "dual_compressed_camera_publisher": camera_base}):
            module = load("vla_dual_camera_publisher")
            node = module.DualVlaCameras()
            self.assertEqual(set(node.views), {"front", "left_wrist", "right_wrist"})
            self.assertEqual(set(topics), {"/camera/front/compressed",
                                          "/camera/left_wrist/compressed",
                                          "/camera/right_wrist/compressed"})
            self.assertEqual(camera_base.CameraCaptureWorker.call_count, 3)
            node.publish_once()
            for side, pub in node.publishers.items():
                self.assertEqual(pub.messages[0].frame_id, side + "_camera")
            camera_base.CameraCaptureWorker.return_value.start.assert_not_called()




# Restore fake import modules between suites during unittest discovery.
from _mock_isolation import isolate_imports
setUpModule, tearDownModule = isolate_imports()

if __name__ == "__main__":
    unittest.main()
