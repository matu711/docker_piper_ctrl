"""Manual CLI checks with all SDK factories, GUI and camera devices replaced."""
import contextlib
import importlib
import io
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, Mock, patch, mock_open

import cv2
import numpy as np

TEST = Path(__file__).resolve().parents[1] / 'test'
sys.path.insert(0, str(TEST))
from _mock_isolation import isolate_imports
setUpModule, tearDownModule = isolate_imports()


class DiagnosticToolsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Even importing the manual tools must not open a camera or CAN bus.
        with patch.object(cv2, 'VideoCapture', side_effect=AssertionError('real camera forbidden')):
            cls.camera = importlib.import_module('test_dual_camera')
            cls.triple = importlib.import_module('test_triple_camera')
            cls.motion = importlib.import_module('test_ctrl_piper_joint')
            cls.reader = importlib.import_module('read_joint_angles')

    def preview(self, module, argv, devices):
        cameras = []
        for _ in devices:
            camera = MagicMock()
            camera.read.return_value = (True, np.zeros((480, 640, 3), dtype=np.uint8))
            camera.get.return_value = 0
            cameras.append(camera)
        with patch.object(sys, 'argv', [module.__file__] + argv), \
                patch.object(cv2, 'VideoCapture', side_effect=cameras) as factory, \
                patch.object(cv2, 'imshow') as display, \
                patch.object(cv2, 'waitKey', return_value=ord('q')), \
                patch.object(cv2, 'destroyAllWindows') as close, \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(module.main(), 0)
            self.assertEqual([call.args[0] for call in factory.call_args_list], devices)
            self.assertEqual(display.call_args[0][1].shape, (480, 640 * len(devices), 3))
            close.assert_called_once()
        for camera in cameras:
            camera.read.assert_called_once()
            camera.release.assert_called_once()
            self.assertTrue(camera.set.called)

    def test_single_camera_preview_opens_only_requested_camera(self):
        self.preview(self.camera, ['--single', '/dev/video1'], ['/dev/video1'])

    def test_two_camera_cli_and_preview_are_preserved(self):
        self.preview(self.camera, [], ['/dev/video6', '/dev/video0'])

    def test_three_camera_cli_and_helper_import_are_preserved(self):
        self.assertIs(self.triple.open_camera, self.camera.open_camera)
        self.preview(self.triple, [], ['/dev/video6', '/dev/video0', '/dev/video2'])

    def test_camera_read_failure_releases_devices(self):
        camera = MagicMock()
        camera.read.return_value = (False, None)
        camera.get.return_value = 0
        with patch.object(sys, 'argv', ['camera', '--single', '/dev/video1']), \
                patch.object(cv2, 'VideoCapture', return_value=camera), \
                patch.object(cv2, 'destroyAllWindows'), \
                contextlib.redirect_stdout(io.StringIO()), self.assertRaises(RuntimeError):
            self.camera.main()
        camera.release.assert_called_once()

    def test_reader_default_cli_keeps_one_shot_and_factory(self):
        state = ((0., .5, -.5, 0., .5, 0.), 10., 200.)
        with patch.object(sys, 'argv', ['read_joint_angles.py', '--can', 'can2']), \
                patch.object(self.reader, 'Piper') as factory, \
                patch.object(self.reader, 'read_updated_state', return_value=state) as read, \
                contextlib.redirect_stdout(io.StringIO()) as output:
            self.reader.main()
        factory.assert_called_once_with('can2')
        read.assert_called_once_with(factory.return_value)
        self.assertIn('受信時刻: 10.0', output.getvalue())

    def test_reader_watch_repeats_until_ctrl_c(self):
        state = ((0.,)*6, 10., 200.)
        with patch.object(sys, 'argv', ['reader', '--can', 'can3', '--watch', '--rate', '5']), \
                patch.object(self.reader, 'Piper'), \
                patch.object(self.reader, 'read_updated_state', return_value=state) as read, \
                patch.object(self.reader.time, 'sleep', side_effect=[None, KeyboardInterrupt]) as sleep, \
                contextlib.redirect_stdout(io.StringIO()):
            self.reader.main()
        self.assertEqual(read.call_count, 2)
        self.assertEqual(sleep.call_args[0], (.2,))

    def test_reader_update_helper_contract_is_preserved(self):
        sdk = Mock()
        state = ((0.,)*6, 11., 200.)
        sdk.get_joint_states.side_effect = [((0.,)*6, 10., 200.), state]
        self.assertEqual(self.reader.read_updated_state(sdk), state)

    def test_motion_default_dry_run_never_executes(self):
        for port in ('can0', 'can1', 'can2', 'can3'):
            with self.subTest(port=port), patch.object(self.motion, 'execute') as execute, \
                    contextlib.redirect_stdout(io.StringIO()) as output:
                self.assertEqual(self.motion.main(['--can', port]), 0)
                execute.assert_not_called()
                self.assertIn('DRY RUN', output.getvalue())
                self.assertIn(port, output.getvalue())

    def test_motion_requires_explicit_execution(self):
        with patch.object(self.motion, 'execute') as execute, contextlib.redirect_stdout(io.StringIO()):
            self.motion.main(['--can', 'can1', '--execute'])
        self.assertEqual(execute.call_args[0][0].can, 'can1')

    def test_motion_helpers_also_refuse_dry_run(self):
        args = self.motion.parse_args(['--can', 'can0'])
        factory = Mock()
        with self.assertRaises(RuntimeError):
            self.motion.run_motion(args, factory, Mock())
        factory.assert_not_called()
        with patch('builtins.open') as open_file, self.assertRaises(RuntimeError):
            self.motion.execute(args)
        open_file.assert_not_called()

    def test_motion_rejects_invalid_parameters_before_sdk(self):
        for args in (['--can', 'can4'], ['--can', 'can0', '--radius', 'nan'],
                     ['--can', 'can0', '--dt', '0'], ['--can', 'can0', '--grip-max', '.2'],
                     ['--can', 'can0', '--speed', '101']):
            with self.subTest(args=args), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                self.motion.parse_args(args)

    def test_motion_waveform_preserves_original_amplitude_and_gripper(self):
        args = self.motion.parse_args(['--can', 'can0'])
        base = [0., .5, -.5, 0., .5, 0.]
        joints, grip = self.motion.motion_target(base, 0., args)
        self.assertEqual(joints, [.15, .5, -.5, 0., .5, 0.])
        self.assertAlmostEqual(grip, .055)
        joints, grip = self.motion.motion_target(base, .5, args)
        self.assertAlmostEqual(joints[1], .65)
        self.assertAlmostEqual(grip, .1)
        self.assertEqual(base, [0., .5, -.5, 0., .5, 0.])

    def run_mock_motion(self, invalid_bounds=False, send_failure=False, bad_mode=False, stale=False):
        args = self.motion.parse_args(['--can', 'can3', '--execute', '--duration', '.01'])
        factory, params = MagicMock(), MagicMock()
        sdk = factory.return_value
        sdk.enable_arm.return_value = True
        sdk.init.return_value.GetArmStatus.return_value = SimpleNamespace(time_stamp=10.,
            arm_status=SimpleNamespace(ctrl_mode=2 if bad_mode else 1, arm_status=0, err_code=0))
        sdk.get_joint_states.return_value = ((0., .5, -.5, 0., .5, 0.), 10. if stale else 11., 200.)
        params.return_value.GetJointLimitParam.return_value = (0., .01) if invalid_bounds else (-3., 3.)
        if send_failure:
            sdk.move_j.side_effect = RuntimeError('synthetic send error')
        if stale:
            args.duration, args.feedback_timeout = 2., .1
        with patch.object(self.reader, 'read_updated_state', return_value=((0., .5, -.5, 0., .5, 0.), 10., 200.)), \
                patch.object(self.motion.time, 'monotonic', side_effect=[0., 0., .2] if stale else [0., 0., 0., .02]), \
                patch.object(self.motion.time, 'sleep'), contextlib.redirect_stdout(io.StringIO()):
            if invalid_bounds or send_failure or bad_mode or stale:
                with self.assertRaises((ValueError, RuntimeError)):
                    self.motion.run_motion(args, factory, params)
            else:
                self.motion.run_motion(args, factory, params)
        factory.assert_called_once_with('can3')
        sdk.init.return_value.EmergencyStop.assert_called_once_with(1)
        sdk.disconnect.assert_called_once()
        return sdk

    def test_motion_sdk_dispatch_and_normal_stop(self):
        sdk = self.run_mock_motion()
        sdk.enable_arm.assert_called_once()
        sdk.move_j.assert_called_once()
        self.assertAlmostEqual(sdk.move_j.call_args[0][0][0], .15, delta=1e-5)
        sdk.move_gripper.assert_called_once_with(.055, 1)

    def test_motion_entire_envelope_checked_before_enable(self):
        sdk = self.run_mock_motion(invalid_bounds=True)
        sdk.enable_arm.assert_not_called()
        sdk.move_j.assert_not_called()

    def test_motion_send_exception_still_stops(self):
        sdk = self.run_mock_motion(send_failure=True)
        sdk.move_gripper.assert_not_called()

    def test_motion_rejects_teaching_mode_and_frozen_feedback(self):
        sdk = self.run_mock_motion(bad_mode=True)
        sdk.enable_arm.assert_not_called()
        sdk.move_j.assert_not_called()
        sdk = self.run_mock_motion(stale=True)
        sdk.move_j.assert_not_called()

    def test_motion_competing_owner_blocks_before_factories(self):
        args = self.motion.parse_args(['--can', 'can0', '--execute'])
        with patch('builtins.open', mock_open()), patch.object(self.motion.fcntl, 'flock'), \
                patch.object(self.motion, 'require_unused_can', side_effect=RuntimeError('busy')), \
                patch.object(self.motion, 'run_motion') as run, self.assertRaisesRegex(RuntimeError, 'busy'):
            self.motion.execute(args)
        run.assert_not_called()

    def test_motion_kernel_owner_check_fails_closed(self):
        fake_path = MagicMock()
        with patch.object(self.motion, 'Path', return_value=fake_path):
            fake_path.glob.return_value = []
            with self.assertRaises(RuntimeError):
                self.motion.require_unused_can('can0')
            file = Mock()
            fake_path.glob.return_value = [file]
            for line in ('can0 123 owner', 'any 123 owner'):
                file.read_text.return_value = line
                with self.assertRaises(RuntimeError):
                    self.motion.require_unused_can('can0')
            file.read_text.return_value = 'can2 123 other'
            self.motion.require_unused_can('can0')


if __name__ == '__main__':
    unittest.main()
