#!/usr/bin/env python3
"""Independent, strict-limit dual initial-pose return; never restarts VLA.

ROS master may be absent. Mixed/ambiguous drivers never fall back to SDK.
--dry-run reads feedback and validates both paths, without enabling or moving.
"""
import argparse
import fcntl
import math
import re
import threading
import time
from pathlib import Path

import rospy
import rosgraph
import yaml
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import Trigger, SetBool

from go_initial_pose import (
    GoInitialPose, STATE_NAMES, validate_target, validate_state,
    validate_pose_limits,
)

SIDES = ("left", "right")
NAMESPACES = {"left": "/followerL", "right": "/followerR"}
RUNTIME_STOP = "/piper_vla_dual_runtime/stop"
CONTROLLER_STOP = "/vla_dual_piper_controller/stop"
RETURN_LOCK = "/vla_dual/initial_pose_lock"
DEFAULTS = dict(move_duration=3.0, command_rate=30.0, z_enabled=True, z_min=0.01,
                gripper_max_width_m=0.10, dh_is_offset=1,
                driver_detection_timeout=2.0, feedback_timeout=1.0,
                arrival_timeout=10.0, sdk_enable_timeout=10.0,
                joint_tolerance_rad=0.02, gripper_tolerance_m=0.002,
                direct_speed_percent=10, feedback_sync_tolerance=0.03,
                command_sync_tolerance=0.03, service_timeout=3.0)


def load_config(path):
    raw = yaml.safe_load(Path(path).expanduser().read_text())
    if not isinstance(raw, dict):
        raise ValueError("YAML must be a mapping")
    cfg = dict(DEFAULTS, **raw)
    if set(cfg["targets"]) != set(SIDES) or set(cfg["can_ports"]) != set(SIDES):
        raise ValueError("targets/can_ports require exactly left and right")
    cfg["targets"] = {s: validate_target(cfg["targets"][s]) for s in SIDES}
    ports = list(cfg["can_ports"].values())
    if len(set(ports)) != 2 or any(not isinstance(p, str) or not re.fullmatch(r"[\w.-]+", p) for p in ports):
        raise ValueError("Two distinct CAN names required")
    if type(cfg["z_enabled"]) is not bool:
        raise ValueError("z_enabled must be bool")
    for name, default in DEFAULTS.items():
        if name == "z_enabled":
            continue
        value = float(cfg[name])
        if not math.isfinite(value):
            raise ValueError(name + " must be finite")
        if name in ("joint_tolerance_rad", "gripper_tolerance_m", "feedback_sync_tolerance"):
            if value < 0:
                raise ValueError(name + " must be nonnegative")
        elif name not in ("z_min", "dh_is_offset") and value <= 0:
            raise ValueError(name + " must be positive")
        cfg[name] = value
    if not 1 <= cfg["direct_speed_percent"] <= 100 or cfg["direct_speed_percent"] % 1:
        raise ValueError("direct_speed_percent must be an integer in [1,100]")
    if not 0 < cfg["gripper_max_width_m"] <= .10:
        raise ValueError("gripper_max_width_m must be in (0,.10]")
    if cfg["dh_is_offset"] not in (0, 1):
        raise ValueError("dh_is_offset must be 0 or 1")
    cfg["direct_speed_percent"] = int(cfg["direct_speed_percent"])
    return cfg


def analyze_topology(state):
    """Require both complete, unique driver routes, or no driver evidence."""
    pubs, subs, services = (dict(group) for group in state)
    routes, evidence, known = {}, set(), set()
    for side, ns in NAMESPACES.items():
        feedback = set(pubs.get(ns + "/joint_states_single", []))
        command = set(subs.get(ns + "/joint_states", []))
        if len(feedback) == 1 and command == feedback:
            node = next(iter(feedback))
            if set(services.get(ns + "/stop_srv", [])) != {node}:
                raise RuntimeError(side + " driver has no unambiguous stop service")
            routes[side] = node
            known.add(node)
        for name in ("joint_states_single", "current_joint_obs", "arm_status", "end_pose", "end_pose_euler"):
            evidence.update(pubs.get(ns + "/" + name, []))
        for name in ("joint_states", "joint_ctrl_single", "pos_cmd", "enable_flag"):
            evidence.update(subs.get(ns + "/" + name, []))
        for name, nodes in services.items():
            if name.startswith(ns + "/"):
                evidence.update(nodes)
    for group in (pubs, subs, services):
        for name, nodes in group.items():
            for node in nodes:
                if any(part in node.lower() for part in (
                    "piper_ctrl_", "piper_dual_follower", "piper_dual_leader",
                    "mcap2_arm")) or node.rsplit("/", 1)[-1] == "dual_piper_control":
                    evidence.add(node)
            if name in ("/joint_states_single", "/current_joint_obs", "/joint_ctrl_single", "/pos_cmd"):
                evidence.update(nodes)
    if routes:
        if set(routes) != set(SIDES) or len(set(routes.values())) != 2 or evidence - known:
            raise RuntimeError("Partial/ambiguous PiPER owners: %s" % sorted(evidence))
        return "ros_driver", routes
    if evidence:
        raise RuntimeError("Driver evidence without both complete routes: %s" % sorted(evidence))
    return "direct_sdk", {}


def require_unused_can(ports):
    """Conservative kernel RX-owner check in this network namespace.

    This cannot discover send-only/unregistered SDK clients. See README: all
    external owners must be stopped; ROS graph + procfs are not a global lease.
    """
    files = list(Path("/proc/net/can").glob("rcvlist_*"))
    if not files:
        raise RuntimeError("Kernel CAN ownership information unavailable; refusing SDK")
    for path in files:
        for line in path.read_text().splitlines():
            fields = line.split()
            if fields and (fields[0] in ports or fields[0] == "any"):
                raise RuntimeError("Existing CAN receiver: " + line.strip())


class Feedback:
    def __init__(self):
        self.condition = threading.Condition()
        self.samples = {}

    def callback(self, message, side):
        with self.condition:
            try:
                if len(message.position) != 7 or set(message.name) != set(STATE_NAMES) or len(message.name) != 7:
                    raise ValueError("Expected joint1..6, gripper exactly once")
                values = validate_state([message.position[message.name.index(n)] for n in STATE_NAMES])
                stamp = message.header.stamp.to_sec()
                if not math.isfinite(stamp) or stamp <= 0:
                    raise ValueError("Invalid feedback timestamp")
                if side in self.samples and stamp < self.samples[side][1]:
                    raise ValueError("Feedback timestamp moved backwards")
                self.samples[side] = (values, stamp, time.monotonic(), None)
            except (ValueError, TypeError) as error:
                self.samples[side] = (None, 0., time.monotonic(), str(error))
            self.condition.notify_all()

    def snapshot(self, timeout, skew, ros_now):
        with self.condition:
            if set(self.samples) != set(SIDES):
                raise RuntimeError("Both feedback streams required")
            for side, (_, stamp, received, error) in self.samples.items():
                if error or time.monotonic() - received > timeout or not 0 <= ros_now - stamp <= timeout:
                    raise RuntimeError(side + ": invalid/stale feedback: " + str(error))
            stamps = [self.samples[s][1] for s in SIDES]
            if max(stamps) - min(stamps) > skew:
                raise RuntimeError("Left/right feedback not synchronized")
            return {s: list(self.samples[s][0]) for s in SIDES}


class DualInitialPose:
    def __init__(self, config, dry_run=False):
        self.cfg, self.dry_run = config, dry_run
        self.ros_active = rosgraph.is_master_online()
        if self.ros_active:
            rospy.init_node("go_dual_initial_pose")
            if rospy.get_param("/use_sim_time", False):
                raise RuntimeError("Hardware return requires wall-clock ROS time")
        self.feedback = Feedback()
        self.hold_pub = None
        self.handles = []
        self.file_locks = []
        self.sdk = {}
        self.backend = None
        self.routes = {}
        self.motion_started = False
        self.lease = False
        self.aborted = threading.Event()
        self.abort_reason = None
        self.control_nodes = set()
        self.sdk_stamps, self.sdk_updates = {}, {}

    def graph(self):
        online = rosgraph.is_master_online()
        if self.ros_active and not online:
            raise RuntimeError("ROS master disappeared; ownership is unknown")
        if not online:
            return ([], [], [])
        if not self.ros_active:
            # A master appearing mid-SDK run changes the ownership assumptions.
            raise RuntimeError("ROS master appeared during SDK return")
        return rosgraph.Master(rospy.get_name()).getSystemState()

    def call(self, service, cls=Trigger, *args):
        """Bound waits without interpreting sleeps as acknowledgement."""
        done, box = threading.Event(), []
        def invoke():
            try:
                rospy.wait_for_service(service, timeout=self.cfg["service_timeout"])
                box.append(rospy.ServiceProxy(service, cls)(*args))
            except Exception as error:
                box.append(error)
            finally:
                done.set()
        threading.Thread(target=invoke, daemon=True).start()
        if not done.wait(self.cfg["service_timeout"]):
            raise RuntimeError("Service timed out; outcome unknown: " + service)
        if isinstance(box[0], Exception):
            raise RuntimeError("Service failed: %s: %s" % (service, box[0]))
        if not box[0].success:
            raise RuntimeError("Service rejected: %s: %s" % (service, box[0].message))
        return box[0]

    def lock_process(self):
        # Serialize this mover across both backends. These advisory locks do
        # not claim that unrelated SDK programs cooperate with them.
        for port in sorted(self.cfg["can_ports"].values()):
            stream = open("/tmp/piper-initial-pose-%s.lock" % port, "a")
            try:
                fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                stream.close()
                raise RuntimeError("Another initial-pose mover owns " + port)
            self.file_locks.append(stream)

    def detect_backend(self):
        deadline = time.monotonic() + self.cfg["driver_detection_timeout"]
        while True:
            mode, routes = analyze_topology(self.graph())
            if mode == "ros_driver":
                for side, node in routes.items():
                    if rospy.get_param(node + "/can_port", None) != self.cfg["can_ports"][side]:
                        raise RuntimeError("Driver CAN setting unknown/mismatch: " + side)
                return mode, routes
            if time.monotonic() >= deadline:
                return mode, routes
            self.pause(.02)

    def guard_callback(self, _message):
        self.abort_reason = "VLA Action arrived during initial-pose ownership"
        self.aborted.set()

    def hold(self):
        if self.hold_pub is not None:
            self.hold_pub.publish(Bool(data=True))

    def prepare_vla(self):
        if not self.ros_active:
            return
        self.hold_pub = rospy.Publisher("/vla_dual/hold", Bool, queue_size=1, latch=True)
        self.handles.append(rospy.Subscriber("/vla_dual/approved_action", Float64MultiArray,
                                             self.guard_callback, queue_size=1))
        self.hold()
        pubs, subs, services = self.graph()
        service_map = dict(services)
        all_nodes = {n for group in (pubs, subs, services) for _, nodes in group for n in nodes}
        if any("piper_vla_dual_runtime" in n for n in all_nodes) and RUNTIME_STOP not in service_map:
            raise RuntimeError("Runtime present without stop service")
        if RUNTIME_STOP in service_map:
            self.call(RUNTIME_STOP)
        controller_nodes = set(service_map.get("/vla_dual/controller_start", []))
        if dict(pubs).get("/vla_dual/approved_action") and RUNTIME_STOP not in service_map:
            raise RuntimeError("Action publisher present without Runtime stop service")
        if controller_nodes:
            if RETURN_LOCK not in service_map or CONTROLLER_STOP not in service_map:
                raise RuntimeError("Controller lacks initial-pose exclusion; update dual Controller")
            self.call(RETURN_LOCK, SetBool, True)
            self.lease = True
            self.call(CONTROLLER_STOP)
        self.control_nodes = controller_nodes
        # Any queued Action observed even during stopping aborts this attempt.
        # A user can retry once the stopped graph is quiescent.
        self.check_guard()

    def check_guard(self):
        if self.aborted.is_set():
            raise RuntimeError(self.abort_reason)
        graph = self.graph()
        current = set(dict(graph[2]).get("/vla_dual/controller_start", []))
        if current != self.control_nodes:
            raise RuntimeError("Controller ownership changed during return")
        self.hold()
        return graph

    def check_backend(self):
        graph = self.check_guard()
        mode, routes = analyze_topology(graph)
        if mode != self.backend or routes != self.routes:
            raise RuntimeError("Driver ownership changed during return")
        if self.ros_active:
            publishers = dict(graph[0])
            for ns in NAMESPACES.values():
                unknown = set(publishers.get(ns + "/joint_states", [])) - self.control_nodes - {rospy.get_name()}
                if unknown:
                    raise RuntimeError("Unknown driver command publishers: %s" % sorted(unknown))

    def pause(self, seconds):
        if self.aborted.wait(max(0., seconds)):
            raise RuntimeError(self.abort_reason)
        if self.ros_active and rospy.is_shutdown():
            raise RuntimeError("ROS shutdown")

    @staticmethod
    def dependencies():
        return GoInitialPose._sdk_dependencies()

    def trajectories(self, start, params, fk):
        result = {}
        for side in SIDES:
            # Reuse unchanged single-arm full-path limit/FK validators.
            mover = GoInitialPose.__new__(GoInitialPose)
            mover.target = self.cfg["targets"][side]
            mover.move_duration = self.cfg["move_duration"]
            mover.command_rate = self.cfg["command_rate"]
            mover.gripper_max_width_m = self.cfg["gripper_max_width_m"]
            mover.z_enabled, mover.z_min = self.cfg["z_enabled"], self.cfg["z_min"]
            result[side] = mover._make_trajectory(start[side], params, fk)
        return result

    def ros_feedback(self):
        return self.feedback.snapshot(self.cfg["feedback_timeout"], self.cfg["feedback_sync_tolerance"],
                                      rospy.Time.now().to_sec())

    def sdk_feedback(self):
        now = time.monotonic()
        vectors, stamps = {}, []
        for side, (piper, interface) in self.sdk.items():
            joints, joint_stamp, _ = piper.get_joint_states()
            grip, grip_stamp, _ = piper.get_gripper_states()
            status = interface.GetArmStatus()
            current = (joint_stamp, grip_stamp, status.time_stamp)
            if any(not math.isfinite(t) or t <= 0 for t in current):
                raise RuntimeError(side + ": SDK feedback not received")
            old = self.sdk_stamps.get(side, (0., 0., 0.))
            updates = self.sdk_updates.setdefault(side, [now] * 3)
            for i, stamp in enumerate(current):
                if stamp < old[i]:
                    raise RuntimeError(side + ": SDK feedback clock moved backwards")
                if stamp > old[i]:
                    updates[i] = now
                if now - updates[i] > self.cfg["feedback_timeout"]:
                    raise RuntimeError(side + ": stale SDK feedback")
            arm = status.arm_status
            initial_teaching = (not self.motion_started and int(arm.arm_status) == 0x0B
                                and int(arm.ctrl_mode) == 2)
            if arm.err_code or (int(arm.arm_status) != 0 and not initial_teaching):
                raise RuntimeError(side + ": SDK hardware fault")
            if self.motion_started and int(status.arm_status.ctrl_mode) != 1:
                raise RuntimeError(side + ": SDK not in CAN control mode")
            self.sdk_stamps[side] = current
            vectors[side] = validate_state(list(joints) + [grip[0]])
            stamps.extend((joint_stamp, grip_stamp))
        if max(stamps) - min(stamps) > self.cfg["feedback_sync_tolerance"]:
            raise RuntimeError("Left/right SDK feedback not synchronized")
        return vectors

    def wait_feedback(self, reader):
        deadline = time.monotonic() + self.cfg["feedback_timeout"]
        while True:
            self.check_backend()
            try:
                return reader()
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                self.pause(.01)

    def stop_both(self):
        """Try both arms independently even if the first stop transport fails."""
        errors = []
        if self.backend == "ros_driver":
            def stop(side):
                try:
                    self.call(NAMESPACES[side] + "/stop_srv")
                except Exception as error:
                    errors.append(side + ": " + str(error))
            threads = [threading.Thread(target=stop, args=(s,), daemon=True) for s in SIDES]
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join(self.cfg["service_timeout"] + .1)
        else:
            for side, (_, interface) in self.sdk.items():
                try:
                    interface.EmergencyStop(0x01)
                except Exception as error:
                    errors.append(side + ": " + str(error))
        if errors:
            rospy.logerr("BOTH-ARM STOP incomplete: %s", "; ".join(errors))

    def execute(self, paths, reader, dispatch):
        started = time.monotonic()
        for index in range(len(paths["left"])):
            deadline = started + (index + 1) / self.cfg["command_rate"]
            self.pause(deadline - time.monotonic())
            if time.monotonic() - deadline > 1. / self.cfg["command_rate"]:
                raise RuntimeError("Missed return control deadline; refusing catch-up burst")
            self.check_backend()
            reader()  # Both feedback streams must be good before either send.
            send_started = time.monotonic()
            self.motion_started = True
            for side in SIDES:
                self.check_guard()
                if time.monotonic() - send_started > self.cfg["command_sync_tolerance"]:
                    raise RuntimeError("Left/right command synchronization exceeded before send")
                dispatch(side, paths[side][index])
            if time.monotonic() - send_started > self.cfg["command_sync_tolerance"]:
                raise RuntimeError("Left/right command synchronization exceeded")
        deadline = time.monotonic() + self.cfg["arrival_timeout"]
        while True:
            self.check_backend()
            actual = reader()
            if all(max(abs(a-b) for a, b in zip(actual[s][:6], self.cfg["targets"][s][:6])) <= self.cfg["joint_tolerance_rad"]
                   and abs(actual[s][6]-self.cfg["targets"][s][6]) <= self.cfg["gripper_tolerance_m"] for s in SIDES):
                return actual
            if time.monotonic() >= deadline:
                raise RuntimeError("Both-arm initial-pose arrival timed out")
            self.pause(.02)

    def _run_ros_driver(self, params, fk):
        for side, ns in NAMESPACES.items():
            self.handles.append(rospy.Subscriber(ns + "/joint_states_single", JointState,
                                                self.feedback.callback, callback_args=side, queue_size=1))
        start = self.wait_feedback(self.ros_feedback)
        paths = self.trajectories(start, params, fk)
        if self.dry_run:
            return paths
        pubs = {s: rospy.Publisher(NAMESPACES[s] + "/joint_states", JointState, queue_size=1) for s in SIDES}
        deadline = time.monotonic() + self.cfg["feedback_timeout"]
        while any(p.get_num_connections() == 0 for p in pubs.values()):
            self.check_backend()
            if time.monotonic() >= deadline:
                raise RuntimeError("Both driver command subscribers required")
            self.pause(.01)
        def dispatch(side, point):
            if pubs[side].get_num_connections() == 0:
                raise RuntimeError(side + ": driver disconnected")
            pubs[side].publish(GoInitialPose._joint_message(point, self.cfg["direct_speed_percent"]))
        return self.execute(paths, self.ros_feedback, dispatch)

    def _run_direct_sdk(self, Piper, params, fk):
        self.check_backend()
        require_unused_can(set(self.cfg["can_ports"].values()))
        for side in SIDES:
            self.check_backend()
            piper = Piper(self.cfg["can_ports"][side])
            interface = piper.init()
            self.sdk[side] = (piper, interface)
            piper.connect()
        start = self.wait_feedback(self.sdk_feedback)
        paths = self.trajectories(start, params, fk)
        if self.dry_run:
            return paths
        # No enable/mode/gripper operation before BOTH full paths validate.
        self.motion_started = True
        deadline = time.monotonic() + self.cfg["sdk_enable_timeout"]
        for side, (piper, interface) in self.sdk.items():
            while not piper.enable_arm():
                self.check_backend()
                if time.monotonic() >= deadline:
                    raise RuntimeError(side + ": enable timed out")
                self.pause(.02)
            piper.enable_gripper()
            while int(interface.GetArmStatus().arm_status.ctrl_mode) != 1:
                self.check_backend()
                if time.monotonic() >= deadline:
                    raise RuntimeError(side + ": CAN mode timed out")
                interface.ModeCtrl(0x01, 0x01, self.cfg["direct_speed_percent"], 0x00)
                self.pause(.02)
        def dispatch(side, point):
            piper, interface = self.sdk[side]
            if not piper.get_ctrl_output_status() or not all(interface.GetArmEnableStatus()):
                raise RuntimeError(side + ": SDK command gate closed")
            piper.move_j(point[:6], self.cfg["direct_speed_percent"])
            piper.move_gripper(point[6], 1)
        return self.execute(paths, self.sdk_feedback, dispatch)

    def run(self):
        try:
            self.lock_process()
            self.prepare_vla()
            self.backend, self.routes = self.detect_backend()
            Piper, ParameterManager, FK = self.dependencies()
            params, fk = ParameterManager(), FK(dh_is_offset=int(self.cfg["dh_is_offset"]))
            for target in self.cfg["targets"].values():
                validate_pose_limits(target, params, self.cfg["gripper_max_width_m"])
            rospy.logwarn("Inter-arm collision checking UNVERIFIED: no calibrated dual collision model available")
            if self.backend == "ros_driver":
                result = self._run_ros_driver(params, fk)
            else:
                result = self._run_direct_sdk(Piper, params, fk)
            rospy.loginfo("Dual initial pose %s via %s; VLA remains stopped",
                          "validated only" if self.dry_run else "reached", self.backend)
            return result
        except BaseException:
            if self.motion_started and not self.dry_run:
                self.stop_both()
            raise
        finally:
            try:
                self.hold()
            except Exception as error:
                rospy.logerr("HOLD publication failed during cleanup: %s", error)
            if self.lease:
                try:
                    self.call(RETURN_LOCK, SetBool, False)
                except Exception as error:
                    rospy.logerr("Return lock release failed; leave Controller HOLD: %s", error)
            # Disconnect only transports opened by THIS process; never disable
            # motors as routine cleanup (loss of torque can drop an arm).
            for piper, _ in self.sdk.values():
                disconnect = getattr(piper, "disconnect", None)
                if disconnect is not None:
                    try:
                        disconnect()
                    except Exception as error:
                        rospy.logerr("SDK disconnect failed: %s", error)
            for stream in self.file_locks:
                stream.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path(__file__).resolve().parent.parent / "config/piper_dual_demo_start.yaml")
    parser.add_argument("--dry-run", action="store_true", help="Read feedback and validate; no enable or motion commands")
    args = parser.parse_args(rospy.myargv(argv=argv)[1:] if argv is not None else rospy.myargv()[1:])
    DualInitialPose(load_config(args.config), args.dry_run).run()


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, ValueError, KeyError, yaml.YAMLError) as error:
        rospy.logerr("Dual initial-pose return aborted: %s", error)
        raise SystemExit(1)
