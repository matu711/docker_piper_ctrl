#!/usr/bin/env python3
"""Mock both complete return backends. Never import an actual SDK or start ROS."""
import copy
import csv
import importlib.util
import os
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1] / "piper"
sys.path.insert(0, str(ROOT / "scripts"))


class Stamp:
    def __init__(self, value): self.value = value
    def to_sec(self): return self.value


class JointState:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None)
        self.name, self.position, self.velocity, self.effort = [], [], [], []


def install():
    ros = types.ModuleType("rospy")
    graph = types.ModuleType("rosgraph")
    sensor = types.ModuleType("sensor_msgs.msg")
    sensor.JointState = JointState
    std = types.ModuleType("std_msgs.msg")
    std.Bool = lambda **kw: types.SimpleNamespace(**kw)
    std.Float64MultiArray = object
    srv = types.ModuleType("std_srvs.srv")
    srv.Trigger, srv.SetBool = object, object
    sys.modules.update(rospy=ros, rosgraph=graph,
                       sensor_msgs=types.ModuleType("sensor_msgs"),
                       std_msgs=types.ModuleType("std_msgs"),
                       std_srvs=types.ModuleType("std_srvs"))
    sys.modules.update({"sensor_msgs.msg": sensor, "std_msgs.msg": std, "std_srvs.srv": srv})
    for name in ("logwarn", "logerr", "loginfo", "init_node", "wait_for_service"):
        setattr(ros, name, Mock())
    ros.is_shutdown = lambda: False
    ros.get_name = lambda: "/go_dual_initial_pose"
    spec = importlib.util.spec_from_file_location("go_dual_initial_pose", ROOT / "scripts/go_dual_initial_pose.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, ros, graph


class Params:
    def GetJointLimitParam(self, name): return (-3., 3.)


class FK:
    def __init__(self, **kwargs): pass
    def CalFK(self, joints): return [[0., 0., 200.]]


class World:
    def __init__(self, module, ros, graph, mode="ros_driver", controller=True):
        self.m, self.ros, self.now, self.mode = module, ros, 1000., mode
        self.controller = controller and mode == "ros_driver"
        self.events, self.commands, self.driver_stops, self.callbacks = [], [], [], {}
        self.positions = {s: [0., .5, -.5, 0., .5, 0., .04] for s in module.SIDES}
        self.graph_override, self.fail_publish, self.slow_publish = None, None, False
        self.fail_stop, self.freeze, self.stick = None, None, False
        self.lease = False
        self.publishers = {}
        self.sdk = {}
        graph.is_master_online = lambda: self.mode != "no_master"
        graph.Master = lambda _: types.SimpleNamespace(getSystemState=self.graph)
        ros.get_param = lambda key, default=None: (
            "can3" if key == "/followerL/driver/can_port" else
            "can1" if key == "/followerR/driver/can_port" else default)
        ros.Time = types.SimpleNamespace(now=lambda: Stamp(self.now))
        ros.Subscriber = self.subscribe
        ros.Publisher = self.publisher
        ros.ServiceProxy = lambda name, cls: lambda *args: self.service(name, args)

    def graph(self):
        self.refresh()
        if self.graph_override is not None: return self.graph_override
        pubs, subs, services = [], [], []
        if self.mode == "ros_driver":
            for side, ns in self.m.NAMESPACES.items():
                node = ns + "/driver"
                pubs.append((ns + "/joint_states_single", [node]))
                subs.append((ns + "/joint_states", [node]))
                services.append((ns + "/stop_srv", [node]))
        if self.controller:
            for service in (self.m.CONTROLLER_STOP, self.m.RETURN_LOCK, "/vla_dual/controller_start"):
                services.append((service, ["/vla_dual_piper_controller"]))
            services.append((self.m.RUNTIME_STOP, ["/piper_vla_dual_runtime"]))
        return pubs, subs, services

    def subscribe(self, topic, cls, callback, **kwargs):
        if "callback_args" in kwargs:
            self.callbacks[kwargs["callback_args"]] = callback
            self.refresh()
        return Mock()

    def refresh(self):
        for side, callback in self.callbacks.items():
            message = JointState()
            message.name = list(self.m.STATE_NAMES)
            message.position = list(self.positions[side])
            message.header.stamp = Stamp(self.now)
            callback(message, side)

    def publisher(self, topic, cls, **kwargs):
        def publish(message):
            for side, ns in self.m.NAMESPACES.items():
                if topic == ns + "/joint_states":
                    if self.fail_publish == side: raise RuntimeError("transport failed")
                    self.commands.append((side, list(message.position), self.now))
                    if not self.stick: self.positions[side] = list(message.position)
                    if self.slow_publish and side == "left": self.now += .1
        value = types.SimpleNamespace(publish=publish, get_num_connections=lambda: 1)
        self.publishers[topic] = value
        return value

    def service(self, name, args):
        self.events.append((name, args))
        if name == self.m.RETURN_LOCK: self.lease = args[0]
        for side, ns in self.m.NAMESPACES.items():
            if name == ns + "/stop_srv":
                self.driver_stops.append(side)
                if self.fail_stop == side: raise RuntimeError("stop failed")
        return types.SimpleNamespace(success=True, message="ok")

    def Piper(self, port):
        side = "left" if port == "can3" else "right"
        world = self
        class Arm:
            def __init__(self):
                self.interface = types.SimpleNamespace(
                    GetArmStatus=lambda: types.SimpleNamespace(time_stamp=world.now,
                        arm_status=types.SimpleNamespace(ctrl_mode=1, err_code=0, arm_status=0)),
                    GetArmEnableStatus=lambda: [True]*6,
                    ModeCtrl=lambda *a: world.events.append(("mode", side)),
                    EmergencyStop=lambda *a: world.driver_stops.append(side))
            def init(self): return self.interface
            def connect(self): world.events.append(("connect", side))
            def disconnect(self): world.events.append(("disconnect", side))
            def get_joint_states(self):
                return list(world.positions[side][:6]), self.feedback_stamp(), None
            def get_gripper_states(self):
                return [world.positions[side][6]], self.feedback_stamp(), None
            def feedback_stamp(self):
                if world.freeze == side and world.commands:
                    if not hasattr(self, "frozen_stamp"): self.frozen_stamp = world.now
                    return self.frozen_stamp
                return world.now
            def enable_arm(self): world.events.append(("enable", side)); return True
            def enable_gripper(self): world.events.append(("enable_gripper", side))
            def get_ctrl_output_status(self): return True
            def move_j(self, joints, speed):
                if world.fail_publish == side: raise RuntimeError("SDK transport failed")
                world.commands.append((side, list(joints), world.now))
                if not world.stick: world.positions[side][:6] = joints
                if world.slow_publish and side == "left": world.now += .1
            def move_gripper(self, grip, force):
                world.events.append(("gripper", side))
                if not world.stick: world.positions[side][6] = grip
        arm = Arm()
        self.sdk[side] = arm
        return arm


class ReturnTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.m, cls.ros, cls.graph = install()

    def run_world(self, mode="ros_driver", dry=False, change=None, fk=FK):
        cfg = self.m.load_config(ROOT / "config/piper_dual_demo_start.yaml")
        world = World(self.m, self.ros, self.graph, mode)
        if change: change(world, cfg)
        mover = self.m.DualInitialPose(cfg, dry)
        mover.lock_process = Mock()
        mover.dependencies = lambda: (world.Piper, Params, fk)
        def pause(seconds):
            if mover.aborted.is_set(): raise RuntimeError(mover.abort_reason)
            world.now += max(0., seconds)
        mover.pause = pause
        with patch.object(self.m.time, "monotonic", lambda: world.now), \
                patch.object(self.m, "require_unused_can") as ownership:
            try:
                result = mover.run()
            except Exception:
                self.world, self.mover = world, mover
                raise
        self.world, self.mover = world, mover
        return result, world, mover, ownership

    def test_ros_return_both_trajectories_and_hold(self):
        _, w, mover, _ = self.run_world()
        self.assertEqual(len(w.commands), 180)
        self.assertEqual([side for side, _, _ in w.commands], ["left", "right"]*90)
        for side in self.m.SIDES:
            self.assertEqual(w.positions[side], mover.cfg["targets"][side])
        self.assertEqual(w.events[0][0], self.m.RUNTIME_STOP)
        self.assertIn((self.m.RETURN_LOCK, (True,)), w.events)
        self.assertIn((self.m.RETURN_LOCK, (False,)), w.events)
        self.assertFalse(w.lease)
        self.assertEqual(w.driver_stops, [])

    def test_sdk_with_master_or_without_master(self):
        for mode in ("no_master", "no_drivers"):
            with self.subTest(mode=mode):
                _, w, mover, check = self.run_world(mode)
                self.assertEqual(len(w.commands), 180)
                check.assert_called_once_with({"can1", "can3"})
                self.assertEqual(w.positions, mover.cfg["targets"])
                self.assertEqual([e for e in w.events if e[0] == "connect"], [("connect", "left"), ("connect", "right")])

    def test_dry_run_never_enables_or_sends_or_stops_hardware(self):
        for mode in ("ros_driver", "no_master", "no_drivers"):
            with self.subTest(mode=mode):
                paths, w, _, _ = self.run_world(mode, True)
                self.assertEqual(len(paths["left"]), 90)
                self.assertEqual(w.commands, [])
                self.assertEqual(w.driver_stops, [])
                self.assertFalse(any(e[0] in ("enable", "enable_gripper", "mode", "gripper") for e in w.events))

    def test_one_driver_or_incomplete_owner_refuses_sdk(self):
        def change(w, cfg):
            pubs, subs, services = w.graph()
            w.graph_override = (pubs[:1], subs[:1], services[:1])
        with self.assertRaises(RuntimeError): self.run_world(change=change)
        self.assertEqual(self.world.sdk, {})
        self.assertEqual(self.world.commands, [])

    def test_unknown_driver_evidence_and_missing_stop_service(self):
        for graph in (([("/other", ["/piper_ctrl_follower_node"])], [], []),
                      ([("/followerL/joint_states_single", ["/x"])], [("/followerL/joint_states", ["/x"])], [])):
            with self.subTest(graph=graph), self.assertRaises(RuntimeError):
                self.m.analyze_topology(graph)

    def test_can_mismatch_refused(self):
        def change(w, cfg): cfg["can_ports"]["left"] = "can9"
        with self.assertRaises(RuntimeError): self.run_world(change=change)
        self.assertEqual(self.world.commands, [])

    def test_full_right_path_invalid_prevents_both_ros_or_sdk_motion(self):
        class BadRight(FK):
            def CalFK(self, joints): return [[0., 0., 0. if joints[0] > .51 else 200.]]
        for mode in ("ros_driver", "no_master"):
            with self.subTest(mode=mode), self.assertRaises(RuntimeError):
                self.run_world(mode, fk=BadRight)
            self.assertEqual(self.world.commands, [])
            self.assertFalse(any(e[0] == "enable" for e in self.world.events))

    def test_transport_failure_attempts_both_stops(self):
        for mode in ("ros_driver", "no_master"):
            def change(w, cfg): w.fail_publish = "right"
            with self.subTest(mode=mode), self.assertRaises(RuntimeError): self.run_world(mode, change=change)
            self.assertEqual(set(self.world.driver_stops), {"left", "right"})

    def test_stop_error_does_not_skip_other_arm(self):
        def change(w, cfg): w.fail_publish, w.fail_stop = "right", "left"
        with self.assertRaises(RuntimeError): self.run_world(change=change)
        self.assertEqual(set(self.world.driver_stops), {"left", "right"})

    def test_sdk_frozen_feedback_stops_both(self):
        def change(w, cfg): w.freeze = "right"
        with self.assertRaises(RuntimeError): self.run_world("no_master", change=change)
        self.assertEqual(set(self.world.driver_stops), {"left", "right"})

    def test_command_skew_aborts_before_right_and_stops_both(self):
        for mode in ("ros_driver", "no_master"):
            def change(w, cfg): w.slow_publish = True
            with self.subTest(mode=mode), self.assertRaises(RuntimeError): self.run_world(mode, change=change)
            self.assertEqual([c[0] for c in self.world.commands], ["left"])
            self.assertEqual(set(self.world.driver_stops), {"left", "right"})

    def test_arrival_timeout_stops_both(self):
        for mode in ("ros_driver", "no_master"):
            def change(w, cfg): w.stick = True
            with self.subTest(mode=mode), self.assertRaises(RuntimeError): self.run_world(mode, change=change)
            self.assertEqual(set(self.world.driver_stops), {"left", "right"})

    def test_feedback_age_skew_and_invalid_values(self):
        f = self.m.Feedback()
        now = self.m.time.monotonic()
        for side in self.m.SIDES: f.samples[side] = ([0.]*7, 100., now, None)
        self.assertEqual(len(f.snapshot(1., .03, 100.)), 2)
        f.samples["right"] = ([0.]*7, 99.9, now, None)
        with self.assertRaises(RuntimeError): f.snapshot(1., .03, 100.)
        f.samples["right"] = ([0.]*7, 100., now-2, None)
        with self.assertRaises(RuntimeError): f.snapshot(1., .03, 100.)

    def test_goal_source_and_defaults(self):
        cfg = self.m.load_config(ROOT / "config/piper_dual_demo_start.yaml")
        self.assertEqual(cfg["move_duration"], 3.)
        self.assertEqual(cfg["command_rate"], 30.)
        self.assertEqual(cfg["can_ports"], {"left": "can3", "right": "can1"})

    def test_goal_matches_optional_recorded_source(self):
        # Real datasets are excluded from Git and Docker build contexts.
        source = os.environ.get('PIPER_INITIAL_POSE_REFERENCE_CSV')
        if not source:
            self.skipTest('Set PIPER_INITIAL_POSE_REFERENCE_CSV for an explicit recorded-pose check')
        source_path = Path(source)
        cfg = self.m.load_config(ROOT / "config/piper_dual_demo_start.yaml")
        with source_path.open() as source:
            row = next(csv.DictReader(source))
        for side in self.m.SIDES:
            self.assertEqual(cfg["targets"][side], [float(row["state_"+side+"_"+k]) for k in self.m.STATE_NAMES])

    def test_can_ownership_unknown_or_busy_is_rejected(self):
        for contents in (None, "can3 000 000 callback user", "any 000 000 callback user"):
            with self.subTest(contents=contents):
                entry = Mock()
                entry.read_text.return_value = contents
                directory = Mock()
                directory.glob.return_value = [] if contents is None else [entry]
                with patch.object(self.m, "Path", return_value=directory), self.assertRaises(RuntimeError):
                    self.m.require_unused_can({"can1", "can3"})
        entry.read_text.return_value = "device filter mask function user\ncan7 000 000 callback user"
        directory.glob.return_value = [entry]
        with patch.object(self.m, "Path", return_value=directory):
            self.m.require_unused_can({"can1", "can3"})

    def test_late_vla_action_aborts_both_without_right_dispatch(self):
        def change(w, cfg):
            original = w.publisher
            def publisher(topic, cls, **kwargs):
                result = original(topic, cls, **kwargs)
                publish = result.publish
                def wrapped(message):
                    publish(message)
                    if topic == "/followerL/joint_states":
                        self.mover.guard_callback(None)
                result.publish = wrapped
                return result
            self.ros.Publisher = publisher
        # Make the callback independent of run_world's exception/result storage.
        original_init = self.m.DualInitialPose.__init__
        def init(mover, *args, **kwargs):
            original_init(mover, *args, **kwargs)
            self.mover = mover
        with patch.object(self.m.DualInitialPose, "__init__", init), self.assertRaises(RuntimeError):
            self.run_world(change=change)
        self.assertEqual([c[0] for c in self.world.commands], ["left"])
        self.assertEqual(set(self.world.driver_stops), {"left", "right"})
        self.assertFalse(self.world.lease)

    def test_ros_feedback_disappearing_during_motion_stops_both(self):
        def change(w, cfg):
            original = w.refresh
            def refresh():
                if not w.commands: original()
            w.refresh = refresh
        with self.assertRaises(RuntimeError): self.run_world(change=change)
        self.assertEqual(set(self.world.driver_stops), {"left", "right"})

    def test_controller_without_exclusion_service_refuses_motion(self):
        def change(w, cfg):
            pubs, subs, services = w.graph()
            w.graph_override = pubs, subs, [(s, n) for s, n in services if s != self.m.RETURN_LOCK]
        with self.assertRaises(RuntimeError): self.run_world(change=change)
        self.assertEqual(self.world.commands, [])
        self.assertEqual(self.world.sdk, {})




# Restore fake import modules between suites during unittest discovery.
from _mock_isolation import isolate_imports
setUpModule, tearDownModule = isolate_imports()

if __name__ == "__main__": unittest.main()
