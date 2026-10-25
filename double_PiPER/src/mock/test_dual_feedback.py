"""No ROS master, sockets or CAN. Tests real dual provenance code and drivers."""
import importlib.util
import json
from pathlib import Path
import sys
import threading
import types
import unittest
from unittest.mock import patch, Mock

from dual_feedback import FeedbackTracker, FeedbackWatch, FRAMES

SCRIPTS = Path(__file__).resolve().parents[1] / 'piper/scripts'


class ProvenanceTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.tracker = FeedbackTracker(lambda: self.now, wall_clock=lambda: self.now)
        self.watch = FeedbackWatch(lambda: self.now)

    def batch(self, exclude=None):
        for key, frame in FRAMES.items():
            if key != exclude:
                self.tracker.received(frame, self.now, 100.+self.now)

    def test_stationary_angles_still_have_real_receive_progress(self):
        self.batch()
        self.assertEqual(self.tracker.take_stamp(.5), 110.)
        self.assertIsNone(self.tracker.take_stamp(.5))
        self.now += .01
        self.batch()
        self.assertAlmostEqual(self.tracker.take_stamp(.5), 110.01)
        self.assertTrue(self.tracker.report(.5)['healthy'])

    def test_each_partial_frame_stop_is_detected_despite_other_traffic(self):
        for missing in FRAMES:
            with self.subTest(frame=missing):
                self.setUp()
                self.batch()
                self.tracker.take_stamp(.5)
                self.now += .6
                self.batch(exclude=missing)
                self.assertFalse(self.tracker.report(.5)['healthy'])
                self.assertIsNone(self.tracker.take_stamp(.5))

    def test_frozen_getters_ros_heartbeat_and_source_stamp_do_not_refresh(self):
        self.batch()
        packet = self.tracker.report(.5)
        self.watch.update('left', packet, .5)
        self.now += .6
        # Lie only in the test fixture as the old ROS publisher did: age=0,
        # heartbeat keeps arriving, but the actual receive counters are frozen.
        for _ in range(3):
            with self.assertRaisesRegex(ValueError, 'stalled'):
                self.watch.update('left', packet, .5)
        self.batch()
        self.watch.update('left', self.tracker.report(.5), .5)
        self.watch.check('left', .5)

    def test_replayed_timestamp_does_not_advance_tracker(self):
        self.batch()
        self.now += 1.
        self.tracker.received(FRAMES['joint12'], 10., 111.)
        self.assertEqual(self.tracker.frames['joint12']['count'], 1)
        self.assertFalse(self.tracker.report(.5)['healthy'])

    def test_socket_backlog_is_not_fresh_even_if_just_decoded(self):
        self.now = 11.
        for frame in FRAMES.values():
            self.tracker.received(frame, 10., 111.)
        self.assertFalse(self.tracker.report(.5)['healthy'])
        self.assertIsNone(self.tracker.take_stamp(.5))

    def test_missing_invalid_reset_and_bus_error_are_rejected(self):
        with self.assertRaises(ValueError): self.watch.check('left', .5)
        self.batch()
        packet=self.tracker.report(.5)
        self.watch.update('left', packet, .5)
        packet['frames']['joint12']['count']=0
        with self.assertRaises(ValueError): self.watch.update('left', packet, .5)
        self.tracker.communication_error=True
        self.assertFalse(self.tracker.report(.5)['healthy'])


class DriverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from test_vla_piper_controller import load_controller_module
        cls.single = load_controller_module()

    def load(self):
        class FakeSDK:
            def ParseCANFrame(self, frame):
                if getattr(frame, 'decode_failure', False):
                    raise ValueError('decode failed')
        fake_sdk = types.ModuleType('piper_sdk')
        fake_sdk.C_PiperInterface=FakeSDK
        fake_node=types.ModuleType('piper_ctrl_single_node')
        fake_node.C_PiperRosNode=object
        with patch.dict(sys.modules, piper_sdk=fake_sdk, piper_ctrl_single_node=fake_node):
            spec=importlib.util.spec_from_file_location('dual_test_driver',SCRIPTS/'piper_ctrl_dual_feedback_node.py')
            module=importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
        return module

    def test_sdk_callback_only_counts_complete_successfully_decoded_frames(self):
        module=self.load()
        module.rospy.Time=types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_sec=lambda:123.))
        sdk=object.__new__(module.TrackedPiperInterface)
        sdk.feedback_lock=threading.RLock()
        sdk.feedback_tracker=FeedbackTracker()
        frame=types.SimpleNamespace(arbitration_id=FRAMES['joint12'], timestamp=__import__('time').time(), data=bytes(8),
                                    is_error_frame=False,is_remote_frame=False,is_extended_id=False)
        sdk.ParseCANFrame(frame)
        self.assertEqual(sdk.feedback_tracker.frames['joint12']['count'],1)
        sdk.ParseCANFrame(frame)
        self.assertEqual(sdk.feedback_tracker.frames['joint12']['count'],1)
        frame.timestamp=__import__('time').time();frame.data=bytes(2)
        sdk.ParseCANFrame(frame)
        self.assertEqual(sdk.feedback_tracker.frames['joint12']['count'],1)
        frame.data=bytes(8);frame.decode_failure=True
        with self.assertRaises(ValueError):sdk.ParseCANFrame(frame)
        self.assertEqual(sdk.feedback_tracker.frames['joint12']['count'],1)

    def test_sdk_communication_status_is_propagated(self):
        module=self.load()
        module.rospy.Time=types.SimpleNamespace(now=lambda:types.SimpleNamespace(to_sec=lambda:123.))
        sdk=object.__new__(module.TrackedPiperInterface)
        sdk.feedback_lock=threading.RLock()
        sdk.feedback_tracker=FeedbackTracker()
        status=types.SimpleNamespace(arm_status=0x05,err_status=types.SimpleNamespace(
            **{'communication_status_joint_%d'%j:False for j in range(1,7)}))
        sdk.GetArmStatus=lambda:types.SimpleNamespace(arm_status=status)
        frame=types.SimpleNamespace(arbitration_id=FRAMES['status'],timestamp=__import__('time').time(),
             data=bytes(8),is_error_frame=False,is_remote_frame=False,is_extended_id=False)
        sdk.ParseCANFrame(frame)
        self.assertTrue(sdk.feedback_tracker.communication_error)
        self.assertFalse(sdk.feedback_tracker.report(.5)['healthy'])

    def test_real_driver_does_not_restamp_cached_state_after_can_stops(self):
        module=self.load()
        module.rospy.Time=types.SimpleNamespace(from_sec=lambda x:x)
        now=[10.]
        tracker=FeedbackTracker(lambda:now[0], wall_clock=lambda:now[0])
        for frame in FRAMES.values():tracker.received(frame,10.,123.)
        joint=types.SimpleNamespace(**{'joint_%d'%j:1000*j for j in range(1,7)})
        grip=types.SimpleNamespace(grippers_angle=40000,grippers_effort=1000)
        high=types.SimpleNamespace(**{'motor_%d'%j:types.SimpleNamespace(motor_speed=0.,effort=0.) for j in range(1,7)})
        sdk=types.SimpleNamespace(feedback_lock=threading.RLock(),feedback_tracker=tracker,
             isOk=lambda:True,GetCanFps=lambda:200., # Deliberately misleading SDK status.
             GetArmJointMsgs=lambda:types.SimpleNamespace(joint_state=joint),
             GetArmGripperMsgs=lambda:types.SimpleNamespace(gripper_state=grip),GetArmHighSpdInfoMsgs=lambda:high)
        driver=object.__new__(module.DualFeedbackNode)
        driver.piper=sdk;driver.feedback_timeout=.5;driver.feedback_pub=Mock();driver.joint_pub=Mock()
        driver.PublishArmJointAndGripper()
        state=driver.joint_pub.publish.call_args.args[0]
        self.assertEqual(state.header.stamp,123.)
        self.assertEqual(state.position[-1],.04)
        self.assertAlmostEqual(state.position[0],.017444)
        for age in (.1,.2,.7):
            now[0]=10.+age
            driver.PublishArmJointAndGripper()
        self.assertEqual(driver.joint_pub.publish.call_count,1)
        self.assertFalse(json.loads(driver.feedback_pub.publish.call_args.args[0])['healthy'])


# Restore fake import modules between suites during unittest discovery.
from _mock_isolation import isolate_imports
setUpModule, tearDownModule = isolate_imports()
