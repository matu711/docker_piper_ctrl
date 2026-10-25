#!/usr/bin/env python3
"""Coordinate four independent legacy-style arm nodes through ROS services."""
import json
import math
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


NODES = ("leaderR", "followerR", "leaderL", "followerL")
FOLLOWERS = ("followerR", "followerL")


class Coordinator:
    def __init__(self):
        rospy.init_node("piper_mcap2_coordinator")
        self.lock = threading.RLock()
        self.operation_lock = threading.Lock()
        self.pool = ThreadPoolExecutor(max_workers=4)
        self.stop_pool = ThreadPoolExecutor(max_workers=2)
        self.statuses, self.vectors = {}, {}
        self.fault = None
        self.returning = False
        self.recovering = False
        self.timeout = float(rospy.get_param("~service_timeout", 20.0))
        self.age = float(rospy.get_param("~feedback_timeout", 1.0))
        self.monitor_feedback = bool(rospy.get_param("~monitor_feedback", False))
        self.stop_both_on_fault = bool(rospy.get_param("~stop_both_on_fault", False))
        self.auto_follow = bool(rospy.get_param("~auto_follow", True))
        self.check_start_alignment = bool(rospy.get_param("~check_start_alignment", False))
        self.joint_tolerance = float(rospy.get_param("~resume_joint_tolerance", .2))
        self.grip_tolerance = float(rospy.get_param("~resume_gripper_tolerance", .005))
        if min(self.timeout, self.age) <= 0 or min(self.joint_tolerance, self.grip_tolerance) < 0:
            raise ValueError("Invalid coordinator timeouts/tolerances")
        for node in NODES:
            rospy.Subscriber("/" + node + "/status", String, self.status_callback, callback_args=node, queue_size=1)
            topic = "joint_grip_pose" if node.startswith("leader") else "current_joint_obs"
            rospy.Subscriber("/" + node + "/" + topic, JointState, self.vector_callback, callback_args=node, queue_size=1)
        self.pub = rospy.Publisher("/dual_piper/status", String, queue_size=1, latch=True)
        for name in ("enable", "enter_teaching_mode", "start_following", "stop_following", "save_initial_pose",
                     "go_initial_pose", "reset_fault", "recover_stop", "emergency_stop"):
            rospy.Service("/dual_piper/" + name, Trigger, self.service(name))
        self.timer = rospy.Timer(rospy.Duration(.02), self.tick)
        rospy.on_shutdown(self.shutdown)

    def status_callback(self, msg, node):
        with self.lock:
            self.statuses[node] = (json.loads(msg.data), time.monotonic())

    def vector_callback(self, msg, node):
        with self.lock:
            self.vectors[node] = (tuple(msg.position), msg.header.stamp.to_sec())

    def call(self, node, operation):
        path = "/%s/%s" % (node, operation)
        rospy.wait_for_service(path, timeout=2.0)
        result = rospy.ServiceProxy(path, Trigger)()
        if not result.success:
            raise RuntimeError("%s: %s" % (node, result.message))
        return result.message

    def both(self, operation, nodes=FOLLOWERS):
        futures = [self.pool.submit(self.call, node, operation) for node in nodes]
        results = []
        deadline = time.monotonic() + self.timeout
        for future in futures:
            results.append(future.result(timeout=max(.001, deadline - time.monotonic())))
        return "; ".join(results)

    def trip(self, reason):
        with self.lock:
            if self.fault:
                return
            self.fault = str(reason)
        rospy.logerr("MCAP2 dual fault: %s", reason)
        # Trigger immediate stops on both independent follower nodes.
        for node in FOLLOWERS:
            self.stop_pool.submit(self.call, node, "emergency_stop")

    def current(self):
        now = time.monotonic()
        with self.lock:
            return {node: state for node, (state, stamp) in self.statuses.items()
                    if not self.monitor_feedback or now - stamp <= self.age}

    def tick(self, _):
        states = self.current()
        problems = ["%s: %s" % (node, state["fault"]) for node, state in states.items() if state.get("fault")]
        # Missing peers during startup do not poison unrelated arm initialization.
        # Once following/returning, loss of a peer must stop both followers.
        following = any(state.get("following") for state in states.values())
        if (following or self.returning) and len(states) != 4:
            problems.append("arm status heartbeat missing")
        if problems and self.stop_both_on_fault and not self.recovering:
            self.trip("; ".join(problems))
        right = states.get("followerR", {}).get("following", False)
        left = states.get("followerL", {}).get("following", False)
        ready = len(states) == 4 and all(state.get("ready") for state in states.values())
        self.pub.publish(String(data=json.dumps({"ready": ready and not self.fault,
            "enabled": len(states) == 4 and all(state.get("enabled") for state in states.values()),
            "following": bool(right and left and not self.fault), "right_following": right,
            "left_following": left, "fault": self.fault, "returning": self.returning, "arms": states,
            "arm_faults": problems, "monitor_feedback": self.monitor_feedback,
            "stop_both_on_fault": self.stop_both_on_fault})))

    def service(self, name):
        def invoke(_):
            if name == "emergency_stop":
                self.trip("Operator emergency stop")
                return TriggerResponse(True, "Both follower stops requested")
            if not self.operation_lock.acquire(False):
                return TriggerResponse(False, "Another dual operation is active")
            try:
                message = getattr(self, name)()
                return TriggerResponse(True, message)
            except Exception as exc:
                # A refused start (alignment/not ready) does not release drives.
                # An enable/home failure after some arms were changed stops both.
                if name in ("enable", "go_initial_pose"):
                    self.trip(str(exc))
                if name == "start_following":
                    try:
                        self.both("stop_following")
                    except Exception:
                        self.trip(str(exc))
                return TriggerResponse(False, str(exc))
            finally:
                self.operation_lock.release()
        return invoke

    def enable(self):
        if self.fault:
            raise RuntimeError("Resolve/reset fault explicitly before enabling")
        message = self.both("enable", NODES)
        if self.auto_follow and not self.check_start_alignment:
            message += "; " + self.both("start_following")
        return message

    def enter_teaching_mode(self):
        if self.fault or self.returning:
            raise RuntimeError("Resolve fault/finish pose return before entering teaching mode")
        return self.both("enter_teaching_mode", ("leaderR", "leaderL"))

    def start_following(self):
        if self.fault or self.returning:
            raise RuntimeError("Resolve fault/finish pose return before following")
        states = self.current()
        if len(states) != 4 or not all(s.get("ready") and s.get("enabled") for s in states.values()):
            raise RuntimeError("Wait for all arm feedback and enable the arms (u)")
        now = rospy.Time.now().to_sec()
        with self.lock:
            for leader, follower in (("leaderR", "followerR"), ("leaderL", "followerL")):
                lv, ls = self.vectors.get(leader, ((), 0))
                fv, fs = self.vectors.get(follower, ((), 0))
                if len(lv) != 7 or len(fv) != 7 or not all(math.isfinite(v) for v in lv + fv):
                    raise RuntimeError("Missing/invalid state vectors")
                if self.monitor_feedback and max(abs(now - ls), abs(now - fs)) > self.age:
                    raise RuntimeError("State vectors stale")
                differences = [abs(a-b) for a, b in zip(lv[:6], fv[:6])]
                grip_target = max(0., min(.1, lv[6]))
                grip_difference = abs(grip_target-fv[6])
                if self.check_start_alignment and (max(differences) > self.joint_tolerance or grip_difference > self.grip_tolerance):
                    index = max(range(6), key=lambda i: differences[i])
                    raise RuntimeError("Align %s and %s before following: J%d difference=%.2f deg, "
                                       "gripper target=%.3f mm measured=%.3f mm difference=%.3f mm" %
                                       (leader, follower, index+1, math.degrees(differences[index]),
                                        grip_target*1000, fv[6]*1000, grip_difference*1000))
        if self.fault:
            raise RuntimeError(self.fault)
        return self.both("start_following")

    def stop_following(self):
        return self.both("stop_following")

    def save_initial_pose(self):
        self.stop_following()
        return self.both("save_initial_pose")

    def go_initial_pose(self):
        if self.fault:
            raise RuntimeError("Resolve fault before returning to initial pose")
        self.stop_following()
        self.returning = True
        try:
            return self.both("go_initial_pose")
        finally:
            self.returning = False

    def reset_fault(self):
        self.recovering = True
        try:
            self.both("reset_fault", NODES)
            self.wait_reset_status()
            return "Fault acknowledged; u to enable, align poses, c to follow"
        finally:
            self.recovering = False

    def recover_stop(self):
        self.recovering = True
        try:
            self.both("recover_stop")
            self.both("reset_fault", ("leaderR", "leaderL"))
            self.wait_reset_status()
            return "Follower stop recovered; u to enable, align poses, c to follow"
        finally:
            self.recovering = False

    def wait_reset_status(self):
        deadline = time.monotonic()+self.timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            states = self.current()
            if len(states) == 4 and all(s.get("ready") and not s.get("fault") and not s.get("following")
                                        for s in states.values()):
                self.fault = None
                return
            time.sleep(.02)
        raise RuntimeError("Waiting for healthy arm status after reset timed out")

    def shutdown(self):
        try:
            self.both("stop_following")
        except Exception:
            pass
        self.pool.shutdown(wait=False)
        self.stop_pool.shutdown(wait=False)


if __name__ == "__main__":
    Coordinator()
    rospy.spin()
