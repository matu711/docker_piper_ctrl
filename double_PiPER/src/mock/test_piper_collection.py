#!/usr/bin/env python3
"""Single/dual contract tests. All ROS endpoints and cameras are fake."""
import ast
import csv
import hashlib
import json
from pathlib import Path
import sys
import time
import unittest
import xml.etree.ElementTree as ET
from unittest.mock import MagicMock, patch
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from mcap.reader import make_reader
from test_piper_collection_reference import FakeCapture, message

BASE = Path(__file__).resolve().parents[1]/'piper'
sys.path.insert(0, str(BASE/'scripts'))
import piper_recorder as recording
import piper_storage as storage
import piper_single as single_adapter
import piper_recording_common as legacy
from piper_config import configuration
from piper_gui import detail_lines


class SingleTests(unittest.TestCase):
    cameras_enabled = True

    def setUp(self):
        import tempfile
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.now = 100.
        self.ros = MagicMock()
        self.ros.is_shutdown.return_value = False
        self.ros.get_param.side_effect = lambda name, default=None: {
            '~configuration': 'single', '~dataset_root': self.directory.name,
            '~enable_cameras': self.cameras_enabled}.get(name, default)
        self.ros.Time.now.side_effect = lambda: rospy.Time.from_sec(self.now)
        self.ros.Time.from_sec.side_effect = rospy.Time.from_sec
        for module in (recording, single_adapter, legacy):
            guard = patch.object(module, 'rospy', self.ros)
            guard.start(); self.addCleanup(guard.stop)
        guard = patch.object(recording, 'CameraCapture', FakeCapture)
        guard.start(); self.addCleanup(guard.stop)
        self.obj = recording.Recorder()
        self.addCleanup(self.obj.shutdown)

    def feed(self, stamp):
        self.obj.sources['single_state'].callback(message(stamp, list(range(6))+[.04]))
        self.obj.sources['single_action'].callback(message(stamp, [.2]*6+[.03]))
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.from_sec(stamp)
        pose.pose.position.x = .123
        pose.pose.orientation.w = 1.
        self.obj.sources['ee'].callback(pose)
        if self.obj.camera_capture:
            self.obj.camera_capture.feed(stamp, round((stamp-100)*20))

    def record_frame(self):
        self.feed(self.now)
        self.obj.start()
        self.now += .2 if self.cameras_enabled else 0
        self.obj.step()

    def test_single_saved_legacy_schema_ee_jpeg_and_mcap(self):
        self.obj.current_task = '袋を持つ'
        self.record_frame()
        self.obj.stop()
        folders = recording.saved_episode_folders(Path(self.directory.name))
        self.assertEqual(len(folders), 1)
        folder = folders[0]
        data = np.load(folder/'data.npz')
        self.assertEqual(data['observation_state'].shape, (1, 7))
        self.assertEqual(data['action'].shape, (1, 7))
        self.assertEqual(data['observation_ee'].shape, (1, 8))
        self.assertEqual(data['observation_state'].dtype, np.float32)
        self.assertEqual(data['unix_timestamp'].dtype, np.float64)
        np.testing.assert_allclose(data['observation_ee'][0], [.123,0,0,0,0,0,1,.04])
        meta = json.loads((folder/'meta.json').read_text())
        self.assertEqual(meta['task'], '袋を持つ')
        self.assertEqual(meta['format'], 'piper_raw_fixed_rate_synced_v6_ee_pose')
        self.assertEqual(meta['observation_ee_source'], '/follower/end_pose')
        with (folder/'frames.csv').open() as stream:
            row = next(csv.DictReader(stream))
        self.assertTrue((folder/row['front_image']).read_bytes().startswith(b'\xff\xd8'))
        self.assertTrue((folder/row['wrist_image']).read_bytes().startswith(b'\xff\xd8'))
        with next(folder.glob('*.mcap')).open('rb') as stream:
            messages = list(make_reader(stream).iter_messages())
        self.assertEqual({channel.topic for _, channel, _ in messages},
                         {'/camera/front/compressed','/camera/wrist/compressed'})
        self.ros.ServiceProxy.assert_not_called()

    def test_exact_single_npz_csv_contract_matches_existing_writer(self):
        self.record_frame()
        episode = self.obj.episode
        from tempfile import TemporaryDirectory
        with TemporaryDirectory() as directory:
            old = legacy.RawEpisodeWriter(Path(directory), 'oracle', episode.args, episode.metadata['names'])
            old.add_frame(b'front', b'wrist', np.array(list(range(6))+[.04]),
                np.array([.2]*6+[.03]), np.array([.123,0,0,0,0,0,1,.04]),
                100.,100.,100.,old.start_monotonic,0.,100.,100.,100.,0.,0.,0.)
            expected = old.save()
            self.obj.stop()
            actual = recording.saved_episode_folders(Path(self.directory.name))[0]
            with np.load(expected/'data.npz') as a, np.load(actual/'data.npz') as b:
                self.assertEqual(a.files, b.files)
                for key in a.files:
                    self.assertEqual(a[key].dtype, b[key].dtype, key)
                    np.testing.assert_allclose(a[key], b[key], atol=1e-6, err_msg=key)
            self.assertEqual((expected/'frames.csv').read_text().splitlines()[0],
                             (actual/'frames.csv').read_text().splitlines()[0])

    def test_terminal_prompt_rules_unicode_empty_cancel_and_snapshot(self):
        with patch.object(recording.sys, 'stdout'):
            self.obj.handle_key('t', source='preview')
            for char in '袋を持つ': self.obj.handle_key(char)
            self.obj.handle_key('\r')
            self.assertEqual(self.obj.current_task, '袋を持つ')
            self.obj.handle_key('t')
            self.obj.handle_key('\r')
            self.assertEqual(self.obj.current_task, '袋を持つ')
            self.obj.handle_key('t')
            self.obj.handle_key('r', source='preview')
            self.assertEqual(self.obj.task_input, '')
            self.obj.handle_key('x')
            self.obj.handle_key('\x1b')
        self.record_frame()
        self.assertFalse(self.obj.begin_task_input())
        self.assertEqual(self.obj.episode.metadata['task'], '袋を持つ')
        self.assertIn('Task: 袋を持つ', detail_lines(self.obj))

    def test_no_control_keys_and_record_save_discard_do_not_call_robot(self):
        for key in ('c',' ','p','h','e','u'):
            self.assertTrue(self.obj.handle_key(key))
            self.assertIsNone(self.obj.key_future)
        self.record_frame()
        self.obj.discard()
        self.assertEqual(recording.saved_episode_folders(Path(self.directory.name)), [])
        self.assertFalse(self.obj.handle_key('q'))
        self.ros.ServiceProxy.assert_not_called()

    def test_real_key_dispatch_r_s_x_and_q(self):
        self.feed(100.)
        self.obj.handle_key('r'); self.obj.key_future.result(timeout=2)
        self.assertIsNotNone(self.obj.episode)
        self.now = 100.2; self.obj.step()
        self.obj.handle_key('s'); self.obj.key_future.result(timeout=2)
        self.assertEqual(self.obj.dataset_count, 1)
        self.now = 101.; self.feed(101.)
        self.obj.handle_key('r'); self.obj.key_future.result(timeout=2)
        self.obj.handle_key('x'); self.obj.key_future.result(timeout=2)
        self.assertIsNone(self.obj.episode)
        self.assertEqual(self.obj.dataset_count, 1)
        self.assertFalse(self.obj.handle_key('q'))

    def test_missing_or_nonfinite_ee_rejects_recording_and_no_fabrication(self):
        self.feed(100.)
        self.obj.sources['ee'].buffer.clear()
        invalid = PoseStamped()
        invalid.header.stamp = rospy.Time.from_sec(100.)
        invalid.pose.position.x = float('nan')
        self.obj.sources['ee'].callback(invalid)
        with self.assertRaisesRegex(RuntimeError, 'Fresh'):
            self.obj.start()
        self.assertIsNone(self.obj.episode)

    def test_single_mcap_keeps_raw_state_action_ee(self):
        self.feed(100.); self.obj.start(); self.feed(100.05)
        self.now = 100.25; self.obj.step(); self.obj.stop()
        folder = recording.saved_episode_folders(Path(self.directory.name))[0]
        with next(folder.glob('*.mcap')).open('rb') as stream:
            topics = {channel.topic for _,channel,_ in make_reader(stream).iter_messages()}
        self.assertTrue({'/follower/current_joint_obs','/follower/applied_joint_cmd','/follower/end_pose'} <= topics)

    def test_shutdown_is_incomplete_and_never_stops_robot(self):
        self.record_frame(); path = self.obj.episode.path
        self.obj.shutdown(); self.obj.shutdown()
        self.assertTrue(list(path.glob('*.incomplete.mcap')))
        self.assertEqual(json.loads((path/'recording_status.json').read_text())['status'], 'incomplete')
        self.ros.ServiceProxy.assert_not_called()


class StateOnlySingleTests(unittest.TestCase):
    cameras_enabled = False
    setUp = SingleTests.setUp
    feed = SingleTests.feed
    record_frame = SingleTests.record_frame

    def test_state_only_retains_ee_without_fake_images(self):
        self.record_frame(); self.obj.stop()
        folder = recording.saved_episode_folders(Path(self.directory.name))[0]
        with np.load(folder/'data.npz') as data:
            self.assertEqual(data['observation_ee'].shape, (1,8))
            self.assertEqual(data['action'].shape, (1,7))
        self.assertEqual(list(folder.glob('*/*.jpg')), [])


class StructureTests(unittest.TestCase):
    def test_launch_switches_ports_cameras_and_same_entrypoints(self):
        for mode, count, ports in [('dual',4,['can0','can1','can2','can3']), ('single',2,['can1','can0'])]:
            root = ET.parse(BASE/'launch'/('collect_'+mode+'_piper_mcap.launch')).getroot()
            args = {a.get('name'): a.get('default') for a in root.findall('arg')}
            nodes = root.findall('group/node')
            self.assertEqual(len(nodes), count)
            self.assertTrue(all(n.get('type') == 'control_piper.py' for n in nodes))
            self.assertEqual([args[n.find("param[@name='can_port']").get('value')[6:-1]] for n in nodes], ports)
            recorder = root.find('node')
            self.assertEqual(recorder.get('type'), 'record_piper_mcap.py')
            self.assertEqual(args['recorder_launch_prefix'], 'xterm -T PiPER-Recorder -e')
            self.assertEqual(recorder.find("param[@name='configuration']").get('value'), mode)
            for node in nodes+[recorder]:
                self.assertTrue((BASE/'scripts'/node.get('type')).stat().st_mode & 0o111)

    def test_no_managed_control_or_robot_service_proxy_in_recorder(self):
        tree = ast.parse((BASE/'scripts/piper_recorder.py').read_text())
        calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)]
        self.assertFalse(any(isinstance(n.func, ast.Attribute) and n.func.attr in ('ServiceProxy','wait_for_service','Piper') for n in calls))
        imports = [n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)]
        self.assertFalse(any('managed' in (name or '') for name in imports))
        self.assertEqual(configuration('single').dimension, 7)
        self.assertEqual(configuration('dual').dimension, 14)


from _mock_isolation import isolate_imports
setUpModule, tearDownModule = isolate_imports()
if __name__ == '__main__': unittest.main()
