#!/usr/bin/env python3
"""Regression checks with synthetic ROS messages; never initializes CAN."""
import ast
import csv
import json
import math
import sys
import tempfile
import time
import unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import JointState, CompressedImage
from mcap.reader import make_reader

BASE = Path(__file__).resolve().parents[1]/"piper"
sys.path.insert(0, str(BASE/"scripts"))
import piper_recorder as recording
import piper_storage as storage
import piper_recording_common as single
import piper_follower as follower


def message(stamp, values):
    msg = JointState()
    msg.header.stamp = rospy.Time.from_sec(stamp)
    msg.position = values
    return msg


class FollowerTests(unittest.TestCase):
    def setUp(self):
        self.node = follower.PiperFollowerNode.__new__(follower.PiperFollowerNode)
        self.node.latest_pose = object()
        self.node.move_spd_rate_ctrl = 40
        self.node.gripper_exist = True
        self.node.piper, self.node.interface, self.node.applied_cmd_pub = MagicMock(), MagicMock(), MagicMock()
        self.node.piper.get_ctrl_output_status.return_value = True
        self.node.interface.get_connect_status.return_value = True
        self.node.interface.GetArmEnableStatus.return_value = [True]*6
        self.node.interface.GetArmStatus.return_value.arm_status.ctrl_mode = 1
        self.ros = MagicMock()
        self.ros.is_shutdown.return_value = False
        self.ros.Time.now.return_value = rospy.Time.from_sec(10.)
        patcher = patch.object(follower, "rospy", self.ros)
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_control_arguments_preserved_and_wire_action_published(self):
        joints = [.123456]*6
        self.node.joint_callback(message(9., joints+[-.0004]))
        self.node.piper.move_j.assert_called_once_with(joints, 40)
        self.node.piper.move_gripper.assert_called_once_with(0., 1)
        sent = self.node.applied_cmd_pub.publish.call_args[0][0]
        self.assertEqual(sent.header.stamp.to_sec(), 10.)
        self.assertAlmostEqual(sent.position[0], math.radians(round(math.degrees(joints[0])*1000)/1000))
        self.assertEqual(sent.position[-1], 0.)

    def test_early_return_does_not_publish_action(self):
        self.node.latest_pose = None
        self.node.joint_callback(message(10., [0.]*7))
        self.node.piper.move_j.assert_not_called()
        self.node.applied_cmd_pub.publish.assert_not_called()

    def test_sdk_skipping_motion_does_not_fabricate_action(self):
        self.node.piper.get_ctrl_output_status.return_value = False
        self.node.joint_callback(message(10., [0.]*7))
        self.node.piper.move_j.assert_called_once()  # Original control path.
        self.node.applied_cmd_pub.publish.assert_not_called()

    def test_publication_error_does_not_escape_control_callback(self):
        self.node.applied_cmd_pub.publish.side_effect = RuntimeError("subscriber unavailable")
        self.node.joint_callback(message(10., [0.]*7))
        self.node.piper.move_j.assert_called_once()
        self.ros.logwarn_throttle.assert_called()

    def test_motion_exception_has_no_action(self):
        self.node.piper.move_gripper.side_effect = RuntimeError("send failed")
        with self.assertRaises(RuntimeError):
            self.node.joint_callback(message(10., [0.]*7))
        self.node.applied_cmd_pub.publish.assert_not_called()

    def test_reference_control_source_identical_except_whitespace(self):
        import hashlib
        self.assertEqual(hashlib.sha256((BASE/"scripts/piper_follower.py").read_bytes()).hexdigest(),
                         '755e32e87e385e102b3549233044ef0ae1840609cd96dcc41ed3d1ff0abb39e1')


class RecorderTests(unittest.TestCase):
    cameras_enabled = False

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.now = 100.
        self.ros = MagicMock()
        self.ros.is_shutdown.return_value = False
        self.ros.get_param.side_effect = lambda name, default=None: {
            "~dataset_root": self.directory.name, "~enable_cameras": self.cameras_enabled}.get(name, default)
        self.ros.Time.now.side_effect = lambda: rospy.Time.from_sec(self.now)
        self.ros.Time.from_sec.side_effect = rospy.Time.from_sec
        camera_patch = patch.object(recording, 'CameraCapture', FakeCapture)
        camera_patch.start()
        self.addCleanup(camera_patch.stop)
        for module in (recording, single):
            patcher = patch.object(module, "rospy", self.ros)
            patcher.start()
            self.addCleanup(patcher.stop)
        self.obj = recording.Recorder()
        self.addCleanup(self.obj.shutdown)

    def feed(self, stamp, omit=None):
        for index, key in enumerate(recording.STREAMS):
            if key != omit:
                self.obj.sources[key].callback(message(stamp, [float(index+1)]*7))

    def test_sync_order_shapes_timestamps_and_readable_files(self):
        self.feed(self.now)
        self.obj.start()
        for index in range(6):
            self.now = 100.+index*.05
            self.feed(self.now)
            self.obj.step()
        result = self.obj.stop()
        folder = next(iter(recording.saved_episode_folders(Path(self.directory.name))))
        self.assertIn("saved:", result)
        with np.load(folder/"data.npz") as data:
            self.assertEqual(data["action"].shape, (6, 14))
            np.testing.assert_array_equal(data["observation_state"][0], [1.]*7+[3.]*7)
            np.testing.assert_array_equal(data["action"][0], [2.]*7+[4.]*7)
            np.testing.assert_allclose(np.diff(data["timestamp"]), .05)
            for key in recording.STREAMS:
                np.testing.assert_allclose(data[key+"_time_error"], 0., atol=2e-9)
        with next(folder.glob("*.mcap")).open("rb") as stream:
            messages = list(make_reader(stream).iter_messages())
        self.assertEqual(len(messages), 24)
        with (folder/"frames.csv").open() as stream:
            self.assertEqual(len(list(csv.DictReader(stream))), 6)
        self.assertEqual(json.loads((folder/"meta.json").read_text())["num_frames"], 6)
        self.assertEqual(self.obj.dataset_count, 1)

    def test_missing_arm_or_invalid_vector_rejects_start(self):
        self.feed(self.now, omit="left_action")
        result = self.obj.service("start")(None)
        self.assertFalse(result.success)
        self.assertIsNone(self.obj.episode)
        for values in ([0.]*6, [float("nan")]*7):
            with self.assertRaises(ValueError):
                recording.vector(message(self.now, values))
        with self.assertRaises(ValueError):
            recording.vector(message(0., [0.]*7))

    def test_sync_error_and_stale_samples(self):
        self.feed(self.now)
        samples = recording.synchronized(self.obj.sources, 100.02, .03)
        self.assertAlmostEqual(samples["right_state"][2], -.02)
        self.assertIsNone(recording.synchronized(self.obj.sources, 100.1, .03))

    def test_missing_data_closes_incomplete_without_controlling_robot(self):
        self.feed(self.now)
        self.obj.start()
        self.obj.step()
        self.now += .1
        self.obj.last_valid = time.monotonic()-2.
        self.obj.step()
        self.assertIsNone(self.obj.episode)
        folder = next(Path(self.directory.name).glob(".tmp_episode_*"))
        metadata = json.loads((folder/"meta.json").read_text())
        self.assertEqual(metadata["status"], "incomplete")
        self.assertEqual(metadata["dropped_frames"]["missing_or_unsynced"], 1)
        self.assertTrue(list(folder.glob("*.incomplete.mcap")))
        self.ros.ServiceProxy.assert_not_called()

    def test_shutdown_preserves_active_data_and_is_idempotent(self):
        self.feed(self.now)
        self.obj.start()
        self.feed(self.now)
        self.obj.step()
        self.obj.shutdown()
        self.obj.shutdown()
        folder = next(Path(self.directory.name).glob(".tmp_episode_*"))
        with next(folder.glob("*.incomplete.mcap")).open("rb") as stream:
            self.assertEqual(len(list(make_reader(stream).iter_messages())), 4)
        with np.load(folder/"data.npz") as data:
            self.assertEqual(data["action"].shape, (1, 14))

    def test_ros_shutdown_hook_during_step_defers_file_finalization(self):
        self.feed(self.now)
        self.obj.start()
        self.obj.step()
        self.now += .05
        self.feed(self.now)
        hook = self.ros.on_shutdown.call_args[0][0]
        with patch.object(self.obj.sources['right_state'], 'wait_until_at_least', side_effect=lambda *args: hook()):
            self.obj.step()
        self.assertIsNotNone(self.obj.episode)
        self.obj.shutdown()
        folder = next(Path(self.directory.name).glob('.tmp_episode_*'))
        with np.load(folder/'data.npz') as data:
            self.assertEqual(data['action'].shape, (1, 14))

    def test_clock_rollback_and_schedule_gap(self):
        self.feed(self.now)
        self.obj.start()
        self.obj.step()
        self.now += .2
        self.feed(self.now)
        self.obj.step()
        self.assertGreaterEqual(self.obj.episode.dropped["schedule"], 2)
        self.now -= .1
        self.obj.step()
        self.assertIsNone(self.obj.episode)

    def test_discard_and_restart(self):
        self.feed(self.now)
        self.obj.start()
        self.obj.discard()
        self.assertFalse(list(Path(self.directory.name).glob(".tmp_episode_*")))
        self.obj.start()
        self.obj.step()
        self.assertIn("saved:", self.obj.stop())


class StorageTests(unittest.TestCase):
    cameras_enabled = False
    setUp = RecorderTests.setUp
    feed = RecorderTests.feed

    def test_default_root_preserved_but_parameter_can_override(self):
        self.assertTrue(any(call.args[0] == "~dataset_root" for call in self.ros.get_param.call_args_list))
        self.assertEqual(recording.DATASET_ROOT, Path.home()/'piper_dataset')
        self.assertEqual(self.obj.root, Path(self.directory.name))

    def test_tokyo_save_names_collisions_and_existing_data_preserved(self):
        from datetime import datetime, timezone
        utc = datetime(2026, 10, 14, 15, 30, 45, tzinfo=timezone.utc)
        with patch.object(storage, 'datetime', wraps=datetime) as clock:
            clock.now.side_effect = lambda tz: utc.astimezone(tz)
            for _ in range(3):
                self.feed(self.now)
                self.obj.start()
                self.obj.step()
                self.obj.stop()
        folders = recording.saved_episode_folders(Path(self.directory.name))
        self.assertEqual([p.name for p in folders], ['20261015_003045', '20261015_003045_01', '20261015_003045_02'])
        self.assertEqual(self.obj.dataset_count, 3)
        for folder in folders:
            with np.load(folder/'data.npz') as data:
                self.assertEqual(data['action'].shape, (1, 14))
        other = recording.Recorder()
        try:
            self.assertEqual(other.dataset_count, 3)
        finally:
            other.shutdown()

    def test_saved_counter_excludes_incomplete_temporary_and_management(self):
        root = Path(self.directory.name)
        for name in ('20261015_010000', 'episode_1791988292530376273', '.tmp_episode_1',
                     'front', 'dual_state', '20261015_010001', '20261015_010002'):
            folder = root/name
            folder.mkdir()
            (folder/'meta.json').write_text(json.dumps(dict(status='saved', num_frames=1)))
            (folder/'data.npz').touch()
            (folder/'frames.csv').touch()
            (folder/'data.mcap').touch()
        (root/'20261015_010001'/'meta.json').write_text(json.dumps(dict(status='incomplete', num_frames=1)))
        (root/'20261015_010002'/'meta.json').write_text('null')
        self.assertEqual([p.name for p in recording.saved_episode_folders(root)],
                         ['20261015_010000', 'episode_1791988292530376273'])

    def test_discard_and_incomplete_do_not_increase_count(self):
        self.feed(self.now)
        self.obj.start()
        self.obj.discard()
        self.assertEqual(self.obj.dataset_count, 0)
        self.obj.start()
        self.obj.step()
        self.obj.shutdown()
        self.assertEqual(len(recording.saved_episode_folders(Path(self.directory.name))), 0)

    def test_default_root_is_portable_and_explicit_directory_is_kept(self):
        self.assertEqual(recording.DATASET_ROOT, Path.home()/'piper_dataset')
        self.assertEqual(self.obj.root, Path(self.directory.name))


class PromptTests(unittest.TestCase):
    cameras_enabled = False
    setUp = RecorderTests.setUp
    feed = RecorderTests.feed

    def enter(self, text):
        with patch.object(recording.sys, 'stdout'):
            self.obj.handle_key('t')
            for key in text:
                self.obj.handle_key(key)
            self.obj.handle_key('\r')

    def test_prompt_persists_and_saved_episodes_have_distinct_tasks(self):
        for task in ('青い袋を持ち上げる', '赤い袋を左へ移動する'):
            self.enter(task)
            self.feed(self.now)
            self.obj.start()
            self.obj.step()
            self.obj.stop()
            self.assertEqual(self.obj.current_task, task)
        folders = recording.saved_episode_folders(Path(self.directory.name))
        tasks = [json.loads((folder/'meta.json').read_text())['task'] for folder in folders]
        self.assertEqual(tasks, ['青い袋を持ち上げる', '赤い袋を左へ移動する'])
        self.feed(self.now)
        self.obj.start()
        self.assertEqual(self.obj.episode.metadata['task'], tasks[-1])

    def test_empty_enter_keeps_previous_and_shortcut_letters_are_text(self):
        self.enter('task qrsxt')
        self.enter('')
        self.assertEqual(self.obj.current_task, 'task qrsxt')
        self.enter('   ')
        self.assertEqual(self.obj.current_task, 'task qrsxt')
        self.assertIsNone(self.obj.episode)

    def test_prompt_change_forbidden_while_recording(self):
        self.enter('initial task')
        self.feed(self.now)
        self.obj.start()
        self.assertFalse(self.obj.begin_task_input())
        self.assertFalse(self.obj.task_input_active)
        self.obj.handle_key('t')
        self.assertEqual(self.obj.current_task, 'initial task')
        self.assertEqual(self.obj.episode.metadata['task'], 'initial task')

    def test_start_rejected_during_input_backspace_and_escape_supported(self):
        self.feed(self.now)
        with patch.object(recording.sys, 'stdout'):
            self.obj.handle_key('t')
            self.assertFalse(self.obj.service('start')(None).success)
            for key in '日本語x':
                self.obj.handle_key(key)
            self.obj.handle_key('\x7f')
            self.obj.handle_key('\r')
            self.assertEqual(self.obj.current_task, '日本語')
            self.obj.handle_key('t')
            self.obj.handle_key('a')
            self.obj.handle_key('\x1b')
        self.assertEqual(self.obj.current_task, '日本語')
        self.assertFalse(self.obj.task_input_active)

    def test_r_snapshots_task_at_keypress_and_disallows_pending_edit(self):
        self.enter('at keypress')
        self.feed(self.now)
        pending = MagicMock()
        pending.done.return_value = False
        with patch.object(self.obj.key_pool, 'submit', return_value=pending) as submit:
            self.obj.handle_key('r')
            self.assertFalse(self.obj.begin_task_input())
            self.obj.current_task = 'later value'
            submit.call_args[0][0]()
        self.assertEqual(self.obj.episode.metadata['task'], 'at keypress')

    def test_launch_task_can_change_within_same_dataset(self):
        self.enter('episode-specific task')
        self.feed(self.now)
        self.obj.start()
        self.obj.step()
        self.obj.stop()
        original = self.ros.get_param.side_effect
        self.ros.get_param.side_effect = lambda name, default=None: 'new launch task' if name == '~task' else original(name, default)
        other = recording.Recorder()
        try:
            self.assertEqual(other.current_task, 'new launch task')
            folder = next(iter(recording.saved_episode_folders(Path(self.directory.name))))
            self.assertEqual(json.loads((folder/'meta.json').read_text())['task'], 'episode-specific task')
        finally:
            other.shutdown()

    def test_preview_keeps_running_without_consuming_task_input(self):
        with patch.object(recording.sys, 'stdout'):
            self.obj.handle_key('t')
            self.obj.handle_key('q', source='preview')
            self.obj.handle_key('r', source='terminal')
            self.assertEqual(self.obj.task_input, 'r')
            self.obj.handle_key('\r')
        self.assertEqual(self.obj.current_task, 'r')
        self.ros.loginfo.assert_any_call('Current task: %s', 'r')


class FakeCapture:
    """Synthetic native-size images: never opens a VideoCapture."""
    def __init__(self, devices, config, worker=None):
        self.frames = {name: deque(maxlen=8) for name, _ in devices}
        self.settings = {name: dict(config['cameras'][name], timestamp_source='synthetic') for name, _ in devices}
        self.closed, self.error = False, None
        self.views = {}
        for name in self.frames:
            self.views[name] = SimpleNamespace(frame_snapshot=lambda name=name: self.snapshot(name), measured_fps=lambda: 30.)
        self.feed(100., 0)

    def feed(self, stamp, sequence, omit=None):
        for name in self.frames:
            if name != omit:
                cfg = self.settings[name]
                frame = np.full((cfg['height'], cfg['width'], 3), 80, np.uint8)
                self.frames[name].append((sequence, frame, stamp, time.monotonic()))

    def snapshot(self, name):
        if self.error:
            raise RuntimeError(self.error)
        return list(self.frames[name])

    def wait_ready(self, *args):
        pass

    def close(self):
        self.closed = True
        return False


class CameraRecorderTests(unittest.TestCase):
    cameras_enabled = True
    setUp = RecorderTests.setUp
    feed = RecorderTests.feed

    def tick(self, index):
        target = 100.+index*.05
        self.feed(target)
        self.obj.camera_capture.feed(target, index)
        self.now = target+.2
        self.obj.step()

    def test_images_mcap_jpeg_npz_csv_and_native_dimensions(self):
        self.feed(100.)
        self.obj.start()
        for index in range(4):
            self.tick(index)
        self.obj.stop()
        folder = next(iter(recording.saved_episode_folders(Path(self.directory.name))))
        with np.load(folder/'data.npz') as data:
            self.assertEqual(data['action'].shape, (4, 14))
            np.testing.assert_array_equal(data['observation_state'][0], [1.]*7+[3.]*7)
            for name in recording.CAMERAS:
                np.testing.assert_array_equal(data[name+'_sequence'], range(4))
                np.testing.assert_allclose(data[name+'_time_error'], 0., atol=2e-9)
                self.assertEqual(len(data[name+'_image']), 4)
            np.testing.assert_allclose(data['camera_sync_span'], 0.)
        for name, shape in (('front', (720, 1280, 3)), ('wrist1', (480, 640, 3)), ('wrist2', (480, 640, 3))):
            images = list((folder/name).glob('*.jpg'))
            self.assertEqual(len(images), 4)
            self.assertEqual(cv2.imread(str(images[0])).shape, shape)
        with next(folder.glob('*.mcap')).open('rb') as stream:
            messages = list(make_reader(stream).iter_messages())
        self.assertEqual(len(messages), 28)
        topics = {channel.topic for _, channel, _ in messages}
        self.assertTrue(all('/camera/'+name+'/compressed' in topics for name in recording.CAMERAS))
        for name in recording.CAMERAS:
            recorded_images = [entry for entry in messages if entry[1].topic == '/camera/'+name+'/compressed']
            for index, (_, _, stored) in enumerate(recorded_images):
                image = CompressedImage()
                image.deserialize(stored.data)
                self.assertAlmostEqual(image.header.stamp.to_sec(), 100.+index*.05)
                decoded = cv2.imdecode(np.frombuffer(image.data, np.uint8), cv2.IMREAD_COLOR)
                self.assertEqual(decoded.shape[:2], (720, 1280) if name == 'front' else (480, 640))
        with (folder/'frames.csv').open() as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(rows[0]['front_image'], 'front/000000.jpg')
        meta = json.loads((folder/'meta.json').read_text())
        self.assertEqual(meta['camera_actual']['front']['width'], 1280)
        self.assertEqual(meta['camera_measured_fps']['wrist2'], 30.)

    def test_missing_old_or_unsynchronized_images_rejected_reuse_allowed(self):
        frame = np.zeros((2, 2, 3), np.uint8)
        def select(stamps, ages=None, last=0, missing=False):
            snapshots = {name: [(1, frame, stamp, 10.-(ages or [0., 0., 0.])[i])]
                         for i, (name, stamp) in enumerate(zip(recording.CAMERAS, stamps))}
            if missing:
                snapshots['wrist2'] = []
            return recording.synchronized_images(snapshots, 100., 100.2, 10.,
                dict.fromkeys(recording.CAMERAS, last), .10, .03, .5)
        self.assertIsNotNone(select([100., 100.01, 100.02]))
        self.assertIsNone(select([100., 100., 100.], missing=True))
        self.assertIsNone(select([99.8, 100., 100.]))
        self.assertIsNone(select([100., 100., 100.], ages=[0., 0., 1.]))
        self.assertIsNotNone(select([100., 100., 100.], last=1))
        self.assertIsNone(select([99.96, 100., 100.04]))
        self.assertIsNone(select([float('nan'), 100., 100.]))

    def test_image_reuse_saved_with_original_timestamps_and_counts(self):
        self.feed(100.)
        self.obj.start()
        self.tick(0)
        self.feed(100.05)
        self.now = 100.25
        self.obj.step()
        self.assertEqual(len(self.obj.episode.rows), 2)
        self.assertEqual(self.obj.episode.dropped['missing_or_unsynced'], 0)
        self.feed(100.2)
        self.now = 100.4
        self.obj.step()
        self.assertEqual(len(self.obj.episode.rows), 2)
        self.assertEqual(self.obj.episode.dropped['missing_or_unsynced'], 1)
        self.obj.stop()
        folder = next(iter(recording.saved_episode_folders(Path(self.directory.name))))
        with np.load(folder/'data.npz') as data:
            for name in recording.CAMERAS:
                np.testing.assert_array_equal(data[name+'_sequence'], [0, 0])
                np.testing.assert_array_equal(data[name+'_reused'], [False, True])
                np.testing.assert_array_equal(data[name+'_reuse_count'], [0, 1])
                np.testing.assert_allclose(data[name+'_ros_timestamp'], [100., 100.])
                np.testing.assert_allclose(data[name+'_time_error'], [0., -.05], atol=2e-9)
        metadata = json.loads((folder/'meta.json').read_text())
        self.assertEqual(metadata['reused_front_frames'], 1)
        self.assertEqual(metadata['reused_wrist1_frames'], 1)
        self.assertEqual(metadata['reused_wrist2_frames'], 1)
        with (folder/'frames.csv').open() as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(rows[1]['front_reuse_count'], '1')

    def test_candidate_scoring_matches_single_arm_worst_error_then_span(self):
        frame = np.zeros((2, 2, 3), np.uint8)
        snapshots = {name: [(0, frame, stamp, 10.)] for name, stamp in
                     zip(recording.CAMERAS, [99.975, 100., 100.])}
        snapshots['front'].append((1, frame, 99.98, 10.))
        snapshots['wrist1'].append((1, frame, 99.98, 10.))
        snapshots['wrist2'].append((1, frame, 99.98, 10.))
        selected = recording.synchronized_images(snapshots, 100., 100.2, 10.,
                    dict.fromkeys(recording.CAMERAS, -1), .1, .03, .5)
        # Sum-error minimization would choose .020 + .000 + .000.
        # Single-arm's worst-error/span scoring prefers the zero-span triple.
        self.assertEqual([selected[name][0] for name in recording.CAMERAS], [1, 1, 1])

    def test_existing_root_metadata_and_old_episode_are_preserved(self):
        path = Path(self.directory.name)/'dataset_meta.json'
        old = dict(self.obj.metadata)
        old.pop('camera_sync_wait')
        old.pop('reuse_count_semantics')
        old['camera_target_tolerance'] = .05
        old['image_semantics'] = 'Native BGR frames encoded as JPEG; no frame reuse; nearest frame within tolerance'
        path.write_text(json.dumps(old))
        episode_path = Path(self.directory.name)/'old_episode'
        episode_path.mkdir()
        (episode_path/'meta.json').write_text(json.dumps(old))
        other = recording.Recorder()
        try:
            self.assertEqual(json.loads(path.read_text()), old)
            self.assertEqual(json.loads((episode_path/'meta.json').read_text()), old)
        finally:
            other.shutdown()

    def test_camera_failure_keeps_readable_incomplete_recording(self):
        self.feed(100.)
        self.obj.start()
        self.tick(0)
        self.obj.camera_capture.error = 'synthetic camera disconnected'
        self.obj.step()
        self.assertIsNone(self.obj.episode)
        folder = next(Path(self.directory.name).glob('.tmp_episode_*'))
        self.assertEqual(json.loads((folder/'meta.json').read_text())['status'], 'incomplete')
        with next(folder.glob('*.incomplete.mcap')).open('rb') as stream:
            self.assertEqual(len(list(make_reader(stream).iter_messages())), 7)
        self.ros.ServiceProxy.assert_not_called()

    def test_encode_failure_does_not_save_a_normal_row(self):
        self.feed(100.)
        self.obj.start()
        with patch.object(recording, 'make_compressed_image', side_effect=RuntimeError('encode failed')):
            self.tick(0)
        folder = next(Path(self.directory.name).glob('.tmp_episode_*'))
        with np.load(folder/'data.npz') as data:
            self.assertEqual(data['action'].shape, (0, 14))
        self.assertEqual(json.loads((folder/'meta.json').read_text())['status'], 'incomplete')

    def test_preview_keys_and_recording_indicator_without_a_gui(self):
        self.feed(100.)
        self.assertTrue(self.obj.handle_key('r'))
        self.obj.key_future.result(timeout=2)
        self.assertIsNotNone(self.obj.episode)
        self.tick(0)
        with patch.object(recording.cv2, 'imshow') as show, \
                patch.object(recording.cv2, 'waitKey', return_value=ord('s')), \
                patch.object(recording.cv2, 'putText', wraps=cv2.putText) as text:
            self.assertEqual(self.obj.preview(), ord('s'))
            self.assertEqual(show.call_args[0][1].shape[1:], (1440, 3))
            self.assertGreater(show.call_args[0][1].shape[0], 270)
            self.assertTrue(any(call.args[1].startswith('RECORDING') for call in text.call_args_list))
            self.assertTrue(any(call.args[1] == 'Dataset Count: 0' for call in text.call_args_list))
        self.obj.handle_key('s')
        self.obj.key_future.result(timeout=2)
        self.assertIsNone(self.obj.episode)
        self.feed(self.now)
        self.obj.camera_capture.feed(self.now, 10)
        self.obj.handle_key('r')
        self.obj.key_future.result(timeout=2)
        self.assertIsNotNone(self.obj.episode)
        self.obj.handle_key('x')
        self.obj.key_future.result(timeout=2)
        self.assertIsNone(self.obj.episode)
        self.assertFalse(self.obj.handle_key('q'))

    def test_actual_resolution_mismatch_is_warned_and_retained(self):
        self.obj.camera_capture.settings['front']['width'] = 1024
        self.obj.refresh_camera_settings()
        self.ros.logwarn.assert_called()
        self.assertEqual(self.obj.camera_settings['front']['width'], 1024)

    def test_shutdown_closes_cameras_after_files(self):
        self.feed(100.)
        self.obj.start()
        self.tick(0)
        capture = self.obj.camera_capture
        def close():
            self.assertIsNone(self.obj.episode)
            folder = next(Path(self.directory.name).glob('.tmp_episode_*'))
            with np.load(folder/'data.npz') as data:
                self.assertEqual(data['action'].shape, (1, 14))
            return False
        with patch.object(capture, 'close', side_effect=close) as closing:
            self.obj.shutdown()
            self.obj.shutdown()
            closing.assert_called_once()

    def test_worker_opens_all_cameras_before_reading_and_preserves_frames(self):
        events, caps = [], []
        stop, connection = recording.threading.Event(), MagicMock()
        connection.send.side_effect = lambda value: stop.set() if value[0] == 'frames' else None
        configs = self.obj.camera_configs
        def open_fake(device, config, event):
            events.append('open '+device)
            cap = MagicMock()
            cap.get.return_value = 0.
            cap.read.side_effect = lambda: (events.append('read '+device) or True,
                                           np.zeros((config['height'], config['width'], 3), np.uint8))
            caps.append(cap)
            return cap, dict(config)
        with patch.object(recording, '_open', side_effect=open_fake), patch.object(recording.signal, 'signal'):
            recording.capture_worker([(name, configs[name]['device']) for name in recording.CAMERAS],
                dict(cameras=configs, fps=30., read_timeout=2.), stop, connection)
        self.assertTrue(all(value.startswith('open ') for value in events[:3]))
        batch = next(call.args[0][1] for call in connection.send.call_args_list if call.args[0][0] == 'frames')
        self.assertEqual(batch['front'][1].shape, (720, 1280, 3))
        self.assertEqual(batch['wrist2'][1].shape, (480, 640, 3))
        for cap in caps:
            cap.release.assert_called_once()

    def test_old_kernel_frame_is_not_relabelled_with_current_capture_time(self):
        stop, connection = recording.threading.Event(), MagicMock()
        connection.send.side_effect = lambda value: stop.set() if value[0] == 'frames' else None
        old_mono = time.monotonic()-5.
        def open_fake(device, config, event):
            cap = MagicMock()
            cap.get.return_value = old_mono*1000
            cap.read.return_value = (True, np.zeros((config['height'], config['width'], 3), np.uint8))
            return cap, dict(config)
        configs = self.obj.camera_configs
        with patch.object(recording, '_open', side_effect=open_fake), patch.object(recording.signal, 'signal'):
            recording.capture_worker([(name, configs[name]['device']) for name in recording.CAMERAS],
                dict(cameras=configs, fps=30., read_timeout=2.), stop, connection)
        frames = next(call.args[0][1] for call in connection.send.call_args_list if call.args[0][0] == 'frames')
        self.assertGreater(time.time()-frames['front'][2], 4.9)
        self.assertGreater(time.monotonic()-frames['front'][3], 4.9)


from _mock_isolation import isolate_imports
setUpModule, tearDownModule = isolate_imports()

if __name__ == "__main__":
    unittest.main()
