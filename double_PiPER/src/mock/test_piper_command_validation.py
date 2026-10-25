"""Pure numerical tests: no ROS, SDK, CAN, camera or filesystem dependencies."""
import copy
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'piper/scripts'))
from piper_command_validation import (
    clip_gripper, parse_follow_command, prepare_sdk_command,
    validate_joint_limits, validate_pose,
)


class CommandValidationTests(unittest.TestCase):
    def setUp(self):
        self.limits = [(-2.6, 2.6), (0., 3.14), (-2.96, 0.),
                       (-1.7, 1.7), (-1.2, 1.2), (-2.09, 2.09)]

    def bounds(self, key):
        return self.limits[int(key[1:]) - 1]

    def pose(self, joints, gripper=.01, style='single'):
        return validate_pose({'joint_positions': joints, 'gripper': gripper},
                             self.bounds, error_style=style)

    def test_parse_numeric_strings_returns_a_new_list(self):
        raw = ['0', '.5', '-.5', '0', '.5', '0', '.01']
        before = raw.copy()
        result = parse_follow_command(raw)
        self.assertEqual(result, [0., .5, -.5, 0., .5, 0., .01])
        self.assertEqual(raw, before)
        self.assertIsNot(raw, result)

    def test_parse_rejects_all_wrong_dimensions(self):
        for size in (0, 1, 2, 3, 4, 5, 6, 8, 9, 100):
            with self.subTest(size=size), self.assertRaisesRegex(ValueError, 'seven'):
                parse_follow_command([0.] * size)

    def test_parse_rejects_nonfinite_and_nonnumeric_at_every_position(self):
        for index in range(7):
            for value in (float('nan'), float('inf'), -float('inf'), 'bad', None):
                raw = [0.] * 7
                raw[index] = value
                with self.subTest(index=index, value=value), self.assertRaises((ValueError, TypeError)):
                    parse_follow_command(raw)

    def test_gripper_clip_and_endpoints(self):
        for raw, expected in ((-.01, 0.), (0., 0.), (.01, .01), (.1, .1), (.11, .1)):
            with self.subTest(raw=raw):
                self.assertEqual(clip_gripper(raw), expected)
        for raw in (float('nan'), float('inf'), -float('inf'), 'bad', None):
            with self.subTest(raw=raw), self.assertRaises((ValueError, TypeError)):
                clip_gripper(raw)

    def test_all_joint_boundaries_and_immediate_neighbours(self):
        for index, (lower, upper) in enumerate(self.limits):
            if index == 1:
                lower = -.05
            if index == 2:
                upper = .05
            for value, accepted in ((lower-1e-9, False), (lower, True), (lower+1e-9, True),
                                    (upper-1e-9, True), (upper, True), (upper+1e-9, False)):
                raw = [0.] * 6
                raw[index] = value
                with self.subTest(joint=index+1, value=value):
                    if accepted:
                        self.assertEqual(validate_joint_limits(raw, self.bounds), raw)
                        for style in ('single', 'dual'):
                            self.assertEqual(self.pose(raw, style=style)[0], raw)
                    else:
                        with self.assertRaises(ValueError):
                            validate_joint_limits(raw, self.bounds)
                        for style in ('single', 'dual'):
                            with self.assertRaises(ValueError):
                                self.pose(raw, style=style)

    def test_zero_margin_is_conditional_and_never_an_offset(self):
        raw = [0., -.05, .05, 0., 0., 0.]
        self.assertEqual(validate_joint_limits(raw, self.bounds), raw)
        with self.assertRaises(ValueError):
            validate_joint_limits(raw, self.bounds, zero_angle_tolerance=0.)
        # Even a nearly-zero bound is not exactly zero and must not expand.
        for index, bounds, value in ((1, (1e-12, 3.), -.001),
                                     (2, (-3., -1e-12), .001)):
            self.limits[index] = bounds
            target = [0.] * 6
            target[1], target[2] = .5, -.5
            target[index] = value
            with self.subTest(index=index), self.assertRaises(ValueError):
                validate_joint_limits(target, self.bounds)

    def test_other_joints_do_not_gain_zero_margin(self):
        for index in (0, 3, 4, 5):
            self.limits[index] = (0., 1.)
            raw = [0.] * 6
            raw[index] = -.001
            with self.subTest(index=index), self.assertRaises(ValueError):
                validate_joint_limits(raw, self.bounds)

    def test_joint_validator_rejects_shape_conversion_and_nonfinite(self):
        for raw in ([0.] * 5, [0.] * 7, None, '000000'):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                validate_joint_limits(raw, self.bounds)
        for value in (float('nan'), float('inf'), -float('inf'), 'bad', None):
            with self.subTest(value=value), self.assertRaises((ValueError, TypeError)):
                validate_joint_limits([value] + [0.] * 5, self.bounds)

    def test_pose_never_clips_gripper(self):
        for style in ('single', 'dual'):
            for grip in (0., .1):
                self.assertEqual(self.pose([0.] * 6, grip, style)[1], grip)
            for grip in (-1e-9, .1+1e-9, float('nan'), float('inf'), -float('inf')):
                with self.subTest(style=style, grip=grip), self.assertRaises(ValueError):
                    self.pose([0.] * 6, grip, style)

    def test_pose_mapping_aliases_and_input_are_preserved(self):
        raw = {'joints': (0., .5, -.5, 0., .5, 0.), 'gripper': '.01', 'metadata': ['keep']}
        before = copy.deepcopy(raw)
        self.assertEqual(validate_pose(raw, self.bounds), (list(raw['joints']), .01))
        self.assertEqual(raw, before)
        for raw in (None, [], {}, {'joint_positions': None, 'joints': [0.]*6, 'gripper': .01}):
            with self.subTest(raw=raw), self.assertRaises((ValueError, TypeError)):
                validate_pose(raw, self.bounds)

    def test_diagnostic_order_matches_each_driver(self):
        raw = {'joint_positions': [100.] + [0.] * 5, 'gripper': .2}
        with self.assertRaisesRegex(ValueError, r'^joint1 100\.000000 rad outside official limit'):
            validate_pose(raw, self.bounds, error_style='single')
        with self.assertRaisesRegex(ValueError, r'^Gripper 200\.000 mm outside'):
            validate_pose(raw, self.bounds, error_style='dual')
        for style, expected in (('single', 'initial pose YAML must be a mapping'),
                                ('dual', 'Pose must be a mapping')):
            with self.assertRaisesRegex(ValueError, '^' + expected + '$'):
                validate_pose(None, self.bounds, error_style=style)

    def test_prepare_rounds_and_does_not_mutate_inputs(self):
        raw = [math.radians(12.34567), .5, -.5, 0., .5, 0.]
        before = raw.copy()
        for style in ('single', 'dual'):
            sent, grip = prepare_sdk_command(raw, .01234567,
                                            lambda value: validate_pose(value, self.bounds, error_style=style))
            self.assertEqual(sent[0], math.radians(12.346))
            self.assertEqual(grip, .012346)
            self.assertEqual(raw, before)
            self.assertIsNot(sent, raw)

    def test_prepare_preserves_python_round_ties(self):
        for degree, expected in ((.0005, 0.), (-.0005, 0.), (.0015, .002)):
            sent, grip = prepare_sdk_command([math.radians(degree)] + [0.] * 5,
                                            .0000005, lambda value: validate_pose(value, self.bounds))
            self.assertEqual(sent[0], math.radians(expected))
            self.assertEqual(grip, 0.)

    def test_prepare_rejects_boundary_crossing_after_rounding(self):
        for index, value in ((1, -.05), (2, .05)):
            raw = [0.] * 6
            raw[index] = value
            for style in ('single', 'dual'):
                before = raw.copy()
                self.assertEqual(self.pose(raw, style=style)[0], raw)
                with self.subTest(index=index, style=style), self.assertRaisesRegex(ValueError, 'joint%d' % (index+1)):
                    prepare_sdk_command(raw, .01, lambda pose: validate_pose(pose, self.bounds, error_style=style))
                self.assertEqual(raw, before)

    def test_prepare_rejects_before_rounding_instead_of_correcting(self):
        for raw, grip in (([3.] + [0.] * 5, .01), ([0.] * 6, .10000001)):
            with self.assertRaises(ValueError):
                prepare_sdk_command(raw, grip, lambda pose: validate_pose(pose, self.bounds))

    def test_prepare_propagates_validation_errors_and_checks_twice(self):
        calls = []
        def validate(value):
            calls.append(copy.deepcopy(value))
            return validate_pose(value, self.bounds)
        prepare_sdk_command([0.] * 6, .01234567, validate)
        self.assertEqual(len(calls), 2)
        self.assertEqual(calls[0]['gripper'], .01234567)
        self.assertEqual(calls[1]['gripper'], .012346)
        def fail(value):
            raise RuntimeError('validation failed')
        with self.assertRaisesRegex(RuntimeError, '^validation failed$'):
            prepare_sdk_command([0.] * 6, .01, fail)




# Restore fake import modules between suites during unittest discovery.
from _mock_isolation import isolate_imports
setUpModule, tearDownModule = isolate_imports()

if __name__ == '__main__':
    unittest.main()
