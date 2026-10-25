#!/usr/bin/env python3
"""MCAP2 offline tests. All arm interfaces and ROS execution are mocked."""
import importlib
import json
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"piper"/"scripts"))
# The tests replace every arm instance with FakePiper below. Do not require a
# particular installed SDK facade just to import the mocked control path.
with patch("piper_sdk.Piper", create=True):
    arm = importlib.import_module("piper_mcap2_arm")
coordinator = importlib.import_module("piper_mcap2_coordinator")
recorder = importlib.import_module("record_dual_piper_mcap2")
camera_module = importlib.import_module("mcap2_camera")


def fake_camera_worker(devices, config, stop, connection):
    connection.send(('settings', {name: {} for name, _ in devices}))
    connection.send(('frames', {name: (0, np.zeros((8, 8, 3), dtype=np.uint8), time.time(), time.monotonic())
                                for name, _ in devices}))
    stop.wait(30.)
    connection.close()


def blocked_camera_worker(devices, config, stop, connection):
    connection.send(('settings', {name: {} for name, _ in devices}))
    connection.send(('frames', {name: (0, np.zeros((8, 8, 3), dtype=np.uint8), time.time(), time.monotonic())
                                for name, _ in devices}))
    time.sleep(30.)  # Simulate a native read that ignores the stop event.


class FakePiper:
    def __init__(self, port):
        self.port, self.mode, self.state, self.error = port, 1, 0, 0
        self.motor_enabled = False
        self.grip_received = True
        self.calls = []
        self.joints = [0.]*6
        self.grip = .01
        self.interface = SimpleNamespace(GetArmStatus=self.status,
            GetArmEnableStatus=lambda: [self.motor_enabled]*6,
            EmergencyStop=self.emergency, MotionCtrl_1=self.motion,
            ModeCtrl=self.mode_ctrl)
    def init(self): return self.interface
    def connect(self): return True
    def status(self):
        return SimpleNamespace(time_stamp=time.time(), arm_status=SimpleNamespace(
            ctrl_mode=self.mode, arm_status=self.state, err_code=self.error))
    def emergency(self, value):
        self.calls.append(('emergency', value))
        self.state = 1 if value == 1 else 0
    def motion(self, *args):
        self.calls.append(('motion', args))
        if args[0] == 2:
            self.state = 0
        if args[2] == 1:
            self.mode, self.state = 2, 0x0B
        elif args[2] == 2:
            self.mode = 0
    def mode_ctrl(self, *args):
        self.calls.append(('mode', args))
        self.mode, self.state = args[0], 0
    def get_joint_states(self): return tuple(self.joints), time.time(), 200
    def get_gripper_states(self): return (self.grip, 0), time.time() if self.grip_received else 0, 200
    def get_end_pose_euler(self): return (0.,)*6, time.time(), 200
    def enable_arm(self):
        self.calls.append(('enable',))
        self.motor_enabled = True
        return True
    def enable_gripper(self):
        self.calls.append(('enable_gripper',))
        self.grip_received = True
    def get_ctrl_output_status(self): return self.motor_enabled
    def move_j(self, values, speed):
        self.calls.append(('move', values, speed))
        self.joints = values
    def move_gripper(self, value, effort):
        self.calls.append(('grip', value, effort))
        self.grip = value


class ArmTests(unittest.TestCase):
    def setUp(self):
        self.ros = MagicMock()
        self.ros.get_param.side_effect = lambda key, default=None: {
            '~role': 'follower', '~can_port': 'can1', '~auto_enable': False,
            '/mcap2_can_ports': ['can0', 'can1', 'can2', 'can3']}.get(key, default)
        self.ros.is_shutdown.return_value = False
        self.ros.Time.now.side_effect = lambda: SimpleNamespace(to_sec=time.time)
        self.ros.Time.from_sec.side_effect = lambda value: SimpleNamespace(to_sec=lambda: value)
        for patcher in (patch.object(arm, 'rospy', self.ros), patch.object(arm, 'Piper', FakePiper),
                        patch.object(arm, 'C_PiperParamManager')):
            value = patcher.start()
            self.addCleanup(patcher.stop)
            if isinstance(value, MagicMock) and value is not self.ros:
                value.return_value.GetJointLimitParam.return_value = (-3., 3.)
        self.node = arm.ArmDriver()

    def test_false_never_enables_or_changes_mode_even_on_tick(self):
        self.node.tick(None)
        self.node.shutdown()
        self.assertEqual(self.node.piper.calls, [])

    def test_enable_first_starts_gripper_feedback(self):
        self.node.piper.grip_received = False
        self.assertTrue(self.node.enable(None).success)
        calls = self.node.piper.calls
        self.assertEqual(calls[0], ('enable',))
        self.assertIn(('enable_gripper',), calls)
        self.assertTrue(self.node.enabled)
        self.assertTrue(self.node.leader_control_blocked)

    def test_mode_switch_is_retried_until_acknowledged(self):
        self.node.piper.mode = 0
        original = self.node.interface.ModeCtrl
        calls = []
        def switch(*args):
            calls.append(args)
            if len(calls) == 3:
                original(*args)
        self.node.interface.ModeCtrl = switch
        self.assertTrue(self.node.enable(None).success)
        self.assertEqual(len(calls), 3)

    def test_stop_available_while_enable_operation_active(self):
        self.node.operation_lock.acquire()
        try:
            result = self.node.service('stop_following')(None)
        finally:
            self.node.operation_lock.release()
        self.assertTrue(result.success)
        self.assertTrue(self.node.enable_cancelled.is_set())
        self.assertTrue(self.node.leader_control_blocked)

    def test_shutdown_is_not_reported_as_can_timeout(self):
        self.ros.is_shutdown.return_value = True
        with self.assertRaisesRegex(RuntimeError, 'cancelled by shutdown'):
            self.node.enable(None)
        self.assertEqual(self.node.piper.calls, [])

    def test_startup_auto_enable_follows_without_alignment_gate(self):
        original = self.ros.get_param.side_effect
        self.ros.get_param.side_effect = lambda key, default=None: True if key == '~auto_enable' else original(key, default)
        node = arm.ArmDriver()
        self.assertTrue(node.enabled)
        self.assertFalse(node.leader_control_blocked)
        message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(to_sec=time.time)),
                                  position=[.5]*6+[.01])
        node.joint_callback(message)
        self.assertIsNone(node.fault)
        self.assertEqual([call for call in node.piper.calls if call[0]=='move'][-1][1],
                         list(node.applied_cmd_pub.publish.call_args[0][0].position[:6]))

    def test_auto_follow_false_keeps_startup_stopped(self):
        original = self.ros.get_param.side_effect
        self.ros.get_param.side_effect = lambda key, default=None: {
            '~auto_enable': True, '~auto_follow': False}.get(key, original(key, default))
        node = arm.ArmDriver()
        self.assertTrue(node.enabled)
        self.assertTrue(node.leader_control_blocked)

    def test_unreceived_joint_feedback_is_rejected(self):
        self.node.piper.get_joint_states = lambda: ((0.,)*6, 0, 0)
        with self.assertRaisesRegex(RuntimeError, 'joints feedback not received'):
            self.node.sample()

    def test_zero_hz_is_not_a_fault_when_timestamp_updates(self):
        self.node.piper.get_joint_states = lambda: ((0.,)*6, time.time(), 0)
        self.node.sample()
        self.node.sample()

    def test_teaching_leader_is_accepted(self):
        self.node.role = 'leader'
        self.node.piper.mode, self.node.piper.state = 2, 0x0B
        self.node.sample()

    def test_stale_feedback_stops_enabled_follower(self):
        self.node.monitor_feedback = True
        self.node.enable(None)
        self.node.sample()
        stamps = list(self.node.last_stamps)
        self.node.piper.get_joint_states = lambda: ((0.,)*6, stamps[0], 200)
        self.node.piper.get_gripper_states = lambda: ((.01, 0), stamps[1], 200)
        self.node.last_updates = [time.monotonic()-2.]*3
        self.node.tick(None)
        self.assertIsNotNone(self.node.fault)
        self.assertIn(('emergency', 1), self.node.piper.calls)
        self.assertTrue(self.node.leader_control_blocked)

    def test_monitoring_off_does_not_stop_on_old_feedback_or_mode(self):
        self.node.enable(None)
        self.node.sample()
        stamps = list(self.node.last_stamps)
        self.node.piper.get_joint_states = lambda: ((0.,)*6, stamps[0], 200)
        self.node.piper.get_gripper_states = lambda: ((.01, 0), stamps[1], 200)
        self.node.last_updates = [time.monotonic()-2.]*3
        self.node.piper.mode = 0
        self.node.leader_control_blocked = False
        self.node.leader_stamp = 0
        self.node.tick(None)
        self.assertFalse(self.node.monitor_feedback)
        self.assertIsNone(self.node.fault)
        self.assertTrue(self.node.enabled)
        self.assertNotIn(('emergency', 1), self.node.piper.calls)

    def test_action_published_matches_quantized_sdk_command(self):
        self.node.enable(None)
        self.node._dispatch_joint_command([.123456]*6, .01234567, 40)
        sent = [entry for entry in self.node.piper.calls if entry[0]=='move'][-1][1]
        message = self.node.applied_cmd_pub.publish.call_args[0][0]
        self.assertEqual(list(message.position[:6]), sent)
        self.assertEqual(message.position[6], .012346)

    def test_both_followers_dispatch_independently_and_keep_local_faults(self):
        right = self.node
        original_params = self.ros.get_param.side_effect
        self.ros.get_param.side_effect = lambda key, default=None: (
            'can3' if key == '~can_port' else original_params(key, default))
        left = arm.ArmDriver()
        self.assertEqual((right.piper.port, left.piper.port), ('can1', 'can3'))
        for node in (right, left):
            # Each real node has its own namespaced publisher; MagicMock's
            # Publisher.return_value would otherwise be shared by both.
            node.applied_cmd_pub = MagicMock()
            node.enable(None)
            node.start_following(None)
            node.piper.calls.clear()
        def command(values):
            return SimpleNamespace(position=values,
                header=SimpleNamespace(stamp=SimpleNamespace(to_sec=time.time)))
        right.joint_callback(command([.2, .5, -.5, 0., .5, 0., .2]))
        self.assertEqual(left.piper.calls, [])
        left.joint_callback(command([-.2, .5, -.5, 0., .5, 0., -.01]))
        for node, expected_joint, expected_grip in ((right, .2, .1), (left, -.2, 0.)):
            sent = [entry for entry in node.piper.calls if entry[0] == 'move']
            self.assertEqual(len(sent), 1)
            self.assertAlmostEqual(sent[0][1][0], expected_joint, delta=1e-5)
            applied = node.applied_cmd_pub.publish.call_args[0][0]
            self.assertEqual(applied.position, sent[0][1] + [expected_grip])
            self.assertIsNone(node.fault)
        left.piper.calls.clear()
        left.joint_callback(command([0.] * 6 + [float('nan')]))
        self.assertIn(('emergency', 1), left.piper.calls)
        self.assertTrue(left.leader_control_blocked)
        self.assertFalse(left.enabled)
        self.assertIsNone(right.fault)
        self.assertTrue(right.enabled)
        with self.assertRaises(RuntimeError):
            left.start_following(None)
        left.joint_callback(command([0.] * 7))
        self.assertFalse(any(entry[0] in ('move', 'grip') for entry in left.piper.calls))

    def test_legacy_gripper_bounds_applied_to_command_and_action(self):
        self.node.enable(None)
        self.node.start_following(None)
        for raw, expected in ((-.001, 0.), (.101, .1)):
            message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(to_sec=time.time)),
                                      position=[0.]*6+[raw])
            self.node.joint_callback(message)
            self.assertIsNone(self.node.fault)
            published = self.node.applied_cmd_pub.publish.call_args[0][0]
            self.assertEqual(published.position[6], expected)
            self.assertEqual([entry for entry in self.node.piper.calls if entry[0]=='grip'][-1][1], expected)

    def test_nonfinite_gripper_still_faults_without_motion(self):
        self.node.enable(None)
        self.node.start_following(None)
        self.node.piper.calls.clear()
        message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(to_sec=time.time)),
                                  position=[0.]*6+[float('nan')])
        self.node.joint_callback(message)
        self.assertIn('Non-finite', self.node.fault)
        self.assertFalse(any(entry[0] in ('move', 'grip') for entry in self.node.piper.calls))

    def test_limits_are_rejected_without_sending(self):
        self.node.enable(None)
        before = len(self.node.piper.calls)
        with self.assertRaises(ValueError):
            self.node._dispatch_joint_command([4.]*6, .01, 40)
        self.assertEqual(len(self.node.piper.calls), before)

    def test_zero_boundary_deviation_is_not_clipped_or_offset(self):
        limits = [(-2.6179, 2.6179), (0, 3.14), (-2.967, 0),
                  (-1.745, 1.745), (-1.22, 1.22), (-2.09439, 2.09439)]
        self.node.params.GetJointLimitParam.side_effect = lambda key: limits[int(key[1:])-1]
        joints = [0., -.036408, .034662, 0., .319011, 0.]
        result, grip = self.node._validate_initial_pose({'joint_positions': joints, 'gripper': .01})
        self.assertEqual(result, joints)
        for index, bad in ((1, -.06), (2, .06), (5, 2.11)):
            invalid = joints.copy()
            invalid[index] = bad
            with self.assertRaises(ValueError):
                self.node._validate_initial_pose({'joint_positions': invalid, 'gripper': .01})
        self.node.zero_angle_tolerance = 0
        with self.assertRaises(ValueError):
            self.node._validate_initial_pose({'joint_positions': joints, 'gripper': .01})

    def test_initial_pose_round_trip_uses_single_arm_format(self):
        self.node.enable(None)
        with tempfile.TemporaryDirectory() as root:
            self.node.initial_pose_file = str(Path(root)/'right.yaml')
            self.assertTrue(self.node.save_initial_pose(None).success)
            self.node.piper.joints = [.1]*6
            self.assertTrue(self.node.go_initial_pose(None).success)
            self.assertTrue(self.node.leader_control_blocked)
            self.assertEqual(self.node.piper.joints, [0.]*6)

    def test_stop_is_recovered_only_explicitly_and_not_enabled(self):
        self.node.piper.state = 1
        with self.assertRaises(RuntimeError):
            self.node.enable(None)
        self.assertEqual(self.node.piper.calls, [])
        self.assertTrue(self.node.recover_stop(None).success)
        self.assertEqual(self.node.piper.calls, [('motion', (2, 6, 2))])
        self.assertFalse(self.node.enabled)
        self.assertTrue(self.node.leader_control_blocked)

    def test_real_error_is_not_cleared_by_recovery(self):
        self.node.piper.state, self.node.piper.error = 1, 1
        with self.assertRaises(RuntimeError):
            self.node.recover_stop(None)
        self.assertEqual(self.node.piper.calls, [])

    def test_healthy_shutdown_does_not_latch_emergency_stop(self):
        self.node.enable(None)
        self.node.shutdown()
        self.assertEqual(self.node.piper.calls[-1], ('motion', (0, 6, 0)))
        self.assertNotIn(('emergency', 1), self.node.piper.calls)


class CoordinatorTests(unittest.TestCase):
    def setUp(self):
        self.obj = coordinator.Coordinator.__new__(coordinator.Coordinator)
        self.obj.lock, self.obj.operation_lock = threading.RLock(), threading.Lock()
        self.obj.pool = MagicMock()
        self.obj.stop_pool = MagicMock()
        self.obj.fault, self.obj.returning, self.obj.recovering = None, False, False
        self.obj.age, self.obj.joint_tolerance, self.obj.grip_tolerance = 1., .2, .005
        self.obj.monitor_feedback, self.obj.stop_both_on_fault = False, False
        self.obj.auto_follow, self.obj.check_start_alignment = True, False
        self.obj.statuses = {name: ({'ready': True, 'enabled': True, 'following': False, 'fault': None},
                                      time.monotonic()) for name in coordinator.NODES}
        self.obj.vectors = {name: ((0.,)*6+(.01,), time.time()) for name in coordinator.NODES}
        self.obj.pub, self.obj.both = MagicMock(), MagicMock(return_value='ok')
        self.ros = MagicMock()
        self.ros.Time.now.return_value.to_sec.return_value = time.time()
        patcher = patch.object(coordinator, 'rospy', self.ros)
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_c_resumes_following_without_changing_leader_mode(self):
        self.obj.start_following()
        self.obj.both.assert_called_once_with('start_following')

    def test_teaching_mode_operation_handles_both_leaders(self):
        self.obj.enter_teaching_mode()
        self.obj.both.assert_called_once_with('enter_teaching_mode', ('leaderR', 'leaderL'))

    def test_default_start_does_not_reject_large_pose_difference(self):
        self.obj.vectors['leaderR'] = ((.5,)*6+(.01,), time.time())
        self.obj.start_following()
        self.obj.both.assert_called_once_with('start_following')

    def test_alignment_failure_never_starts_following(self):
        self.obj.check_start_alignment = True
        self.obj.vectors['leaderL'] = ((.5,)*6+(.01,), time.time())
        with self.assertRaisesRegex(RuntimeError, 'Align'):
            self.obj.start_following()
        self.obj.both.assert_not_called()

    def test_one_fault_stops_both_and_never_auto_resumes(self):
        self.obj.stop_both_on_fault = True
        self.obj.statuses['followerL'][0]['fault'] = 'stale'
        self.obj.tick(None)
        self.assertEqual([call.args[0] for call in self.obj.stop_pool.submit.call_args_list],
                         [self.obj.call, self.obj.call])
        self.assertEqual([call.args[1] for call in self.obj.stop_pool.submit.call_args_list],
                         ['followerR', 'followerL'])
        self.obj.statuses['followerL'][0]['fault'] = None
        self.obj.tick(None)
        self.assertIsNotNone(self.obj.fault)

    def test_arm_only_mode_does_not_propagate_one_fault(self):
        self.obj.statuses['followerL'][0]['fault'] = 'local arm error'
        self.obj.statuses['followerR'][0]['following'] = True
        self.obj.tick(None)
        self.obj.stop_pool.submit.assert_not_called()
        self.assertIsNone(self.obj.fault)
        status = json.loads(self.obj.pub.publish.call_args[0][0].data)
        self.assertTrue(status['right_following'])
        self.assertEqual(status['arm_faults'], ['followerL: local arm error'])

    def test_home_failure_stops_both(self):
        self.obj.both.side_effect = ["stopped", RuntimeError('left home timeout')]
        result = self.obj.service('go_initial_pose')(None)
        self.assertFalse(result.success)
        self.assertIsNotNone(self.obj.fault)
        self.assertFalse(self.obj.returning)


class DatasetTests(unittest.TestCase):
    def args(self, root):
        return SimpleNamespace(root=Path(root), fps=20, camera_fps=30, task='test', repo_id='local/test',
            sync_tolerance=.03, camera_sync_tolerance=.03, camera_target_tolerance=.1)

    def test_images_require_all_three_and_bounded_skew(self):
        item = lambda stamp: (0, None, stamp, stamp)
        self.assertIsNone(recorder.select_images([[item(1)], [], [item(1)]], 1, .03, .1))
        self.assertIsNone(recorder.select_images([[item(.94)], [item(1)], [item(1.06)]], 1, .03, .1))
        self.assertIsNotNone(recorder.select_images([[item(1)], [item(1.01)], [item(1.02)]], 1, .03, .1))

    def test_r_prepares_teaching_then_following_then_waits_for_data(self):
        instance = recorder.Recorder.__new__(recorder.Recorder)
        instance.lock = threading.RLock()
        instance.episode = None
        instance.control = MagicMock()
        instance.healthy = MagicMock(side_effect=RuntimeError('missing state/action'))
        with patch.object(recorder.time, 'monotonic', side_effect=[0., 3.]):
            with self.assertRaisesRegex(RuntimeError, 'missing state/action'):
                instance.start()
        self.assertEqual([call.args[0] for call in instance.control.call_args_list],
                         ['enter_teaching_mode', 'start_following', 'stop_following'])

    def test_npz_csv_jpeg_order_timestamps_and_gaps(self):
        with tempfile.TemporaryDirectory() as root:
            episode = recorder.Episode(self.args(root))
            streams = {name: (np.full(7, 1 if name.startswith('right') else 2), 10., 0.) for name in recorder.STREAMS}
            images = [(0, None, 10., time.monotonic())]*3
            episode.add(0, 10., 10., streams, images, [b'jpeg']*3)
            episode.dropped['missing_or_unsynced'] = 1
            episode.add(2, 10.1, 10., streams, images, [b'jpeg']*3)
            path = episode.finish()
            with np.load(path/'data.npz') as data:
                self.assertEqual(data['observation_state'].shape, (2, 14))
                self.assertEqual(data['action'][0].tolist(), [1]*7+[2]*7)
                self.assertEqual(data['frame_index'].tolist(), [0, 2])
                self.assertIn('left_action_time_error', data)
            self.assertTrue((path/'frames.csv').exists())
            for name in recorder.CAMERAS:
                self.assertEqual((path/name/'000002.jpg').read_bytes(), b'jpeg')
            meta = json.loads((path/'meta.json').read_text())
            self.assertEqual(meta['dropped_frames']['missing_or_unsynced'], 1)

    def test_mcap_contains_both_states_actions_and_three_images(self):
        from mcap.reader import make_reader
        from sensor_msgs.msg import JointState, CompressedImage
        import rospy
        self.assertIsNotNone(recorder.McapRos1Writer)
        with tempfile.TemporaryDirectory() as root:
            writer = recorder.McapEpisodeRecorder()
            writer.start(Path(root))
            topics = ['/follower'+suffix+'/'+leaf for suffix in ('R', 'L')
                      for leaf in ('current_joint_obs', 'applied_joint_cmd')]
            for topic in topics:
                message = JointState()
                message.header.stamp = rospy.Time.from_sec(10.)
                message.position = [0.]*7
                writer.write_ros_message(topic, message)
            for name in recorder.CAMERAS:
                topic = '/camera/'+name+'/compressed'
                topics.append(topic)
                message = CompressedImage()
                message.header.stamp = rospy.Time.from_sec(10.)
                message.format, message.data = 'jpeg', b'jpeg'
                writer.write_ros_message(topic, message)
            output = writer.finish()
            with output.open('rb') as stream:
                recorded = [channel.topic for _, channel, _ in make_reader(stream).iter_messages()]
            self.assertEqual(set(recorded), set(topics))

    def test_shutdown_finalizes_readable_mcap_npz_csv_before_capture_close(self):
        from mcap.reader import make_reader
        from sensor_msgs.msg import JointState
        with tempfile.TemporaryDirectory() as root:
            args = self.args(root)
            args.no_preview = True
            instance = recorder.Recorder.__new__(recorder.Recorder)
            instance.args, instance.lock = args, threading.RLock()
            instance.closing, instance.shutdown_lock = threading.Event(), threading.Lock()
            instance.closed = False
            instance.episode = recorder.Episode(args)
            episode = instance.episode
            streams = {name: (np.zeros(7), 10., 0.) for name in recorder.STREAMS}
            images = [(0, None, 10., time.monotonic())]*3
            episode.add(0, 10., 10., streams, images, [b'jpeg']*3)
            instance.mcap = recorder.McapEpisodeRecorder()
            instance.mcap.start(episode.path)
            instance.mcap.write_ros_message('/test/state', JointState())
            instance.control = MagicMock(side_effect=RuntimeError('service unavailable during shutdown'))
            instance.camera_capture = MagicMock()
            def check_files_first():
                self.assertIsNone(instance.episode)
                self.assertFalse(instance.mcap.active)
                with np.load(episode.path/'data.npz') as data:
                    self.assertEqual(data['action'].shape, (1, 14))
                return False
            instance.camera_capture.close.side_effect = check_files_first
            with patch.object(recorder, 'rospy', MagicMock()):
                instance.shutdown()
                instance.shutdown()
            instance.camera_capture.close.assert_called_once()
            with next(episode.path.glob('*.incomplete.mcap')).open('rb') as stream:
                self.assertEqual(len(list(make_reader(stream).iter_messages())), 1)
            self.assertTrue((episode.path/'frames.csv').exists())
            self.assertEqual(json.loads((episode.path/'meta.json').read_text())['status'], 'incomplete')
            self.assertEqual(episode.finish('incomplete'), episode.path)

    def test_blocked_control_service_has_bounded_wait(self):
        instance = recorder.Recorder.__new__(recorder.Recorder)
        instance.closing = threading.Event()
        instance.args = SimpleNamespace(control_timeout=.05)
        ros = MagicMock()
        done = threading.Event()
        ros.ServiceProxy.return_value.side_effect = lambda: done.wait(5.)
        started = time.monotonic()
        try:
            with patch.object(recorder, 'rospy', ros):
                with self.assertRaisesRegex(RuntimeError, 'Control service timed out'):
                    instance.control('stop_following')
            self.assertLess(time.monotonic()-started, .5)
        finally:
            done.set()


class CameraTests(unittest.TestCase):
    def config(self):
        return dict(width=640, height=480, fps=30, fourcc='MJPG', backend='v4l2',
                    open_timeout=1., read_timeout=2.)

    def test_initialization_has_no_warmup_reads(self):
        cap = MagicMock()
        cap.isOpened.return_value = True
        cap.get.side_effect = lambda prop: {camera_module.cv2.CAP_PROP_FRAME_WIDTH: 1280,
            camera_module.cv2.CAP_PROP_FRAME_HEIGHT: 720, camera_module.cv2.CAP_PROP_FPS: 30,
            camera_module.cv2.CAP_PROP_FOURCC: camera_module.cv2.VideoWriter_fourcc(*'MJPG')}.get(prop, 0)
        with patch.object(camera_module.cv2, 'VideoCapture', return_value=cap):
            result, settings = camera_module._open('/dev/test', self.config(), threading.Event())
        self.assertIs(result, cap)
        cap.read.assert_not_called()
        self.assertEqual((settings['width'], settings['height']), (1280, 720))
        self.assertEqual((settings['output_width'], settings['output_height']), (640, 480))

    def test_frame_timestamp_clock_domains_and_fallback(self):
        cap = MagicMock()
        cap.get.return_value = 10020.
        stamp, mono, source = camera_module.frame_time(cap, 1000., 10., 1000.1, 10.1, 2.)
        self.assertAlmostEqual(stamp, 1000.02)
        self.assertAlmostEqual(mono, 10.02)
        self.assertEqual(source, 'v4l2_monotonic')
        cap.get.return_value = 1000020.
        self.assertEqual(camera_module.frame_time(cap, 1000., 10., 1000.1, 10.1, 2.)[2], 'v4l2_realtime')
        cap.get.return_value = 0
        self.assertEqual(camera_module.frame_time(cap, 1000., 10., 1000.1, 10.1, 2.)[2], 'read_midpoint_fallback')

    def test_all_cameras_open_before_sequential_reads_and_release(self):
        events = []
        stop = threading.Event()
        connection = MagicMock()
        connection.send.side_effect = lambda value: stop.set() if value[0]=='frames' else None
        caps = {}
        def create(device, config, event):
            events.append('open '+device)
            cap = MagicMock()
            cap.read.side_effect = lambda: (events.append('read '+device) or True, np.zeros((480, 640, 3), dtype=np.uint8))
            caps[device] = cap
            return cap, {}
        devices = [('front', 'f'), ('wrist1', 'w1'), ('wrist2', 'w2')]
        with patch.object(camera_module, '_open', side_effect=create), patch.object(camera_module.signal, 'signal'):
            camera_module._camera_worker(devices, self.config(), stop, connection)
        self.assertEqual(events, ['open f', 'open w1', 'open w2', 'read f', 'read w1', 'read w2'])
        for cap in caps.values():
            cap.release.assert_called_once()

    def test_stuck_first_stream_is_reopened_on_its_owner(self):
        stop = threading.Event()
        connection = MagicMock()
        connection.send.side_effect = lambda value: stop.set() if value[0] == 'frames' else None
        stuck, recovered = MagicMock(), MagicMock()
        stuck.read.return_value = (False, None)
        recovered.read.return_value = (True, np.zeros((480, 640, 3), dtype=np.uint8))
        with patch.object(camera_module, '_open', side_effect=[(stuck, {}), (recovered, {})]) as opener, \
                patch.object(camera_module.signal, 'signal'):
            camera_module._camera_worker([('front', 'fake')], self.config(), stop, connection)
        self.assertEqual(opener.call_count, 2)
        stuck.release.assert_called_once()
        recovered.release.assert_called_once()
        self.assertTrue(any(call.args[0][0] == 'frames' for call in connection.send.call_args_list))
        self.assertFalse(any(call.args[0][0] == 'error' for call in connection.send.call_args_list))

    def test_camera_process_closes_gracefully(self):
        capture = camera_module.CameraCapture([('front', 'fake')], self.config(), worker=fake_camera_worker)
        try:
            capture.wait_ready(5.)
            self.assertEqual(len(capture.views['front'].frame_snapshot()), 1)
        finally:
            self.assertFalse(capture.close())
        self.assertFalse(capture.close())

    def test_stuck_native_read_cannot_block_parent_cleanup(self):
        capture = camera_module.CameraCapture([('front', 'fake')], self.config(), worker=blocked_camera_worker)
        try:
            capture.wait_ready(5.)
        finally:
            started = time.monotonic()
            self.assertTrue(capture.close())
            self.assertLess(time.monotonic()-started, 2.5)




# Restore fake import modules between suites during unittest discovery.
from _mock_isolation import isolate_imports
setUpModule, tearDownModule = isolate_imports()

if __name__ == '__main__':
    unittest.main()
