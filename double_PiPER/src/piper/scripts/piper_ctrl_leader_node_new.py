#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import time

import rospy
import numpy as np
import yaml
from sensor_msgs.msg import JointState
from piper_msgs.msg import PiperEulerPose
from piper_sdk import Piper
from piper_sdk.piper_param.piper_param_manager import C_PiperParamManager
from std_srvs.srv import Trigger, TriggerResponse



class PiperLeaderNode:

    def __init__(self):

        rospy.init_node("piper_ctrl_leader_node")

        # =========================
        # params
        # =========================
        self.can_port = rospy.get_param("~can_port", "can1")
        self.auto_enable = rospy.get_param("~auto_enable", True)
        self.gripper_exist = rospy.get_param("~gripper_exist", True)

        self.move_spd_rate_ctrl = 40
        self.have_gripper = True
        self.initial_pose_file = os.path.expanduser(rospy.get_param(
            "~initial_pose_file", "~/.config/piper/initial_pose.yaml"))
        self.initial_pose_speed = int(rospy.get_param("~initial_pose_speed", 20))
        self.initial_pose_timeout = float(rospy.get_param("~initial_pose_timeout", 15.0))
        self.initial_pose_joint_tolerance = float(rospy.get_param("~initial_pose_joint_tolerance", 0.035))
        self.initial_pose_gripper_tolerance = float(rospy.get_param("~initial_pose_gripper_tolerance", 0.005))
        self.initial_pose_mode_timeout = float(rospy.get_param("~initial_pose_mode_timeout", 3.0))
        self.params = C_PiperParamManager()
        if not 1 <= self.initial_pose_speed <= 100:
            raise ValueError("~initial_pose_speed must be in 1..100")

        # =========================
        # Workspace limit (meters)
        # =========================
        # self.workspace = {
        #     "x_min":  -0.05,
        #     "x_max":   0.05,
        #     "y_min":  -0.32,
        #     "y_max":   0.32,
        #     "z_min":   0.1,
        #     "z_max":   0.65,
        #     "rz_min": -3.0,
        #     "rz_max":  3.0,
        # }
        # self.workspace = {
        #     "x_min": -math.inf,
        #     "x_max":  math.inf,
        #     "y_min": -math.inf,
        #     "y_max":  math.inf,
        #     "z_min": -math.inf,
        #     "z_max":  math.inf,
        # }

        # =========================
        # Piper init
        # =========================
        self.piper = Piper(self.can_port)
        self.interface = self.piper.init()

        self.piper.connect()
        time.sleep(0.2)


        # =========================
        # 自動モードに切り替えるための初期化プロセス
        # =========================
        while not self.piper.enable_arm():
            time.sleep(0.01)

        self.piper.enable_gripper()

        if self.interface.GetArmStatus().arm_status.ctrl_mode != 1:
            self.stop()  # This function must be called first when exiting the teaching mode for the first time to switch to CAN mode
        over_time = time.time() + 10.0
        while self.interface.GetArmStatus().arm_status.ctrl_mode != 1:
            if over_time < time.time():
                print("ERROR: Failed to switch to CAN mode, please check if the teaching mode is exited")
                exit()
            self.interface.ModeCtrl(0x01, 0x01, self.move_spd_rate_ctrl, 0x00)
            time.sleep(0.01)
        self.enable()

        while not self.piper.enable_arm():
            time.sleep(0.01)

        self.piper.enable_gripper()

        ### ------

        if self.auto_enable:
            while not self.piper.enable_arm():
                time.sleep(0.01)

            if self.gripper_exist:
                self.piper.enable_gripper()

        self.interface.ModeCtrl(0x01, 0x01, self.move_spd_rate_ctrl, 0x00)

        rospy.loginfo("Leader:" + str(self.can_port) + " ready.")

        print("\n===============================")
        print("Leader Left:"+ str(self.can_port))
        print("===============================")

        # =========================
        # Publisher
        # =========================
        self.joint_pose_pub = rospy.Publisher(
            "joint_grip_pose",
            JointState,
            queue_size=1
        )
        # self.end_pose_pub = rospy.Publisher(
        #     "end_pose_euler",
        #     PiperEulerPose,
        #     queue_size=1
        # )
        self.end_pose_pub2 = rospy.Publisher(
            "end_pose_euler",
            JointState,
            queue_size=1
        )
        # 定期publish用timer
        rospy.Timer(rospy.Duration(0.02), self.Publish_JointPos)
        # rospy.Timer(rospy.Duration(0.02), self.Publish_EndPos)
        rospy.Timer(rospy.Duration(0.02), self.Publish_EndPos2)

        # =========================
        # Service
        # =========================
        self.go_home_srv = rospy.Service(
            "go_home",
            Trigger,
            self.Service_Go_Home
        )
        self.go_initial_pose_srv = rospy.Service(
            "go_initial_pose", Trigger, self.Service_Go_Initial_Pose
        )
        self.enter_teaching_mode_srv = rospy.Service(
            "enter_teaching_mode", Trigger, self.Service_Enter_Teaching_Mode
        )
    
    # =========================================================
    # Joint角度＆グリッパー位置を取得＆公開
    # =========================================================
    def Publish_JointPos(self, event):
        joint_pos = self.piper.get_joint_states()[0]
        gripper_pos = self.piper.get_gripper_states()[0][0]
        # end_pos = self.piper.get_end_pose_euler()[0]

        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = [
            "joint1","joint2","joint3",
            "joint4","joint5","joint6",
            "gripper"
        ]

        msg.position = list(joint_pos) + [gripper_pos]
        self.joint_pose_pub.publish(msg)
    
    # =========================================================
    # エンドエフェクタ位の位置と角度を取得＆公開
    # =========================================================
    # def Publish_EndPos(self, event):
    #     # joint_pos = self.piper.get_joint_states()[0]
    #     # gripper_pos = self.piper.get_gripper_states()[0][0]
    #     end_pos = self.piper.get_end_pose_euler()[0]

    #     end_pose_euler = PiperEulerPose()

    #     end_pose_euler.header.stamp = rospy.Time.now()
    #     end_pose_euler.x = end_pos[0]
    #     end_pose_euler.y = end_pos[1]
    #     end_pose_euler.z = end_pos[2]
    #     end_pose_euler.roll  = end_pos[3]
    #     end_pose_euler.pitch = end_pos[4]
    #     end_pose_euler.yaw   = end_pos[5]
    #     self.end_pose_pub.publish(end_pose_euler)
    
    def Publish_EndPos2(self, event):
        # joint_pos = self.piper.get_joint_states()[0]
        # gripper_pos = self.piper.get_gripper_states()[0][0]
        end_pos = self.piper.get_end_pose_euler()[0]

        end_pose_euler = JointState()
        end_pose_euler.header.stamp = rospy.Time.now()
        end_pose_euler.name = [
            "x","y","z",
            "roll","pitch","yaw"
        ]
        end_pose_euler.position = end_pos

        self.end_pose_pub2.publish(end_pose_euler)

    # =========================================================
    # ホームポジションに戻すサービス
    # =========================================================
    def Service_Go_Home(self, req):
        rospy.loginfo("Service: Go Home Position!")

        while not self.piper.enable_arm():
            time.sleep(0.01)

        self.piper.enable_gripper()

        if self.interface.GetArmStatus().arm_status.ctrl_mode != 1:
            self.stop()  # This function must be called first when exiting the teaching mode for the first time to switch to CAN mode
        over_time = time.time() + 10.0
        while self.interface.GetArmStatus().arm_status.ctrl_mode != 1:
            if over_time < time.time():
                print("ERROR: Failed to switch to CAN mode, please check if the teaching mode is exited")
                exit()
            self.interface.ModeCtrl(0x01, 0x01, self.move_spd_rate_ctrl, 0x00)
            time.sleep(0.01)
        self.enable()

        while not self.piper.enable_arm():
            time.sleep(0.01)

        self.piper.enable_gripper()

        joints=np.zeros(6)
        joints[0] = -1.57
        joints[1] =  0
        joints[2] =  0
        joints[3] =  0
        joints[4] = -0
        joints[5] = -2
        grip_pos = 0

        self.piper.move_j(joints, self.move_spd_rate_ctrl)
        self.piper.move_gripper(grip_pos, 1)

        # print("===================")
        # print("Go Home position")
        # print("===================")
        # rospy.loginfo("Go Home position")

        return TriggerResponse(
        success=True,
        message="Robot moved to home position"
    )

    def _validate_initial_pose(self, value):
        if not isinstance(value, dict):
            raise ValueError("initial pose YAML must be a mapping")
        joints = value.get("joint_positions", value.get("joints"))
        gripper = value.get("gripper")
        if not isinstance(joints, (list, tuple)) or len(joints) != 6:
            raise ValueError("initial pose must contain six joint positions")
        joints, gripper = [float(item) for item in joints], float(gripper)
        if not all(math.isfinite(item) for item in joints + [gripper]):
            raise ValueError("initial pose contains NaN/inf")
        for index, joint in enumerate(joints, start=1):
            lower, upper = self.params.GetJointLimitParam("j%d" % index)
            if not lower <= joint <= upper:
                raise ValueError("joint%d is outside official SDK limits" % index)
        if not 0.0 <= gripper <= 0.1:
            raise ValueError("gripper is outside [0.0, 0.1] m")
        return joints, gripper

    def Service_Go_Initial_Pose(self, req):
        try:
            with open(self.initial_pose_file, "r", encoding="utf-8") as stream:
                joints, gripper = self._validate_initial_pose(yaml.safe_load(stream))
        except Exception as exc:
            return TriggerResponse(False, "Could not load initial pose: {}".format(exc))
        try:
            # Do not call disable_arm(): it removes drive torque and can let
            # a gravity-loaded leader drop.  The official drag-teach command
            # exits teaching while keeping the arm enabled.
            if not self._enter_can_control_for_initial_pose():
                return TriggerResponse(False, "Leader did not enter CAN control; no motion was sent")
            self.piper.move_j(joints, self.initial_pose_speed)
            if self.gripper_exist:
                self.piper.move_gripper(gripper, 1)
        except Exception as exc:
            return TriggerResponse(False, "Leader initial-pose command failed: {}".format(exc))
        deadline = time.monotonic() + self.initial_pose_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                measured_joints = self.piper.get_joint_states()[0]
                measured_gripper = self.piper.get_gripper_states()[0][0]
                joint_error = max(abs(now - target) for now, target in zip(measured_joints, joints))
                if joint_error <= self.initial_pose_joint_tolerance and abs(measured_gripper - gripper) <= self.initial_pose_gripper_tolerance:
                    return TriggerResponse(True, "Leader reached initial pose in CAN control; press r with a hand on the leader to enter TEACHING_MODE")
            except Exception as exc:
                rospy.logwarn_throttle(1.0, "Waiting for leader initial pose: %s", exc)
            time.sleep(0.05)
        return TriggerResponse(False, "Leader initial-pose return timed out")

    def Service_Enter_Teaching_Mode(self, req):
        if int(self.interface.GetArmStatus().arm_status.ctrl_mode) == 0x02:
            return TriggerResponse(True, "Leader is already in TEACHING_MODE")
        if self._return_to_teaching_mode():
            return TriggerResponse(True, "Leader entered TEACHING_MODE")
        return TriggerResponse(False, "Leader did not enter TEACHING_MODE")

    def _wait_ctrl_mode(self, expected_mode):
        deadline = time.monotonic() + self.initial_pose_mode_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if int(self.interface.GetArmStatus().arm_status.ctrl_mode) == expected_mode:
                return True
            time.sleep(0.05)
        return False

    def _wait_until_not_teaching(self):
        deadline = time.monotonic() + self.initial_pose_mode_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if int(self.interface.GetArmStatus().arm_status.ctrl_mode) != 0x02:
                return True
            time.sleep(0.05)
        return False

    def _enter_can_control_for_initial_pose(self):
        current_mode = int(self.interface.GetArmStatus().arm_status.ctrl_mode)
        if current_mode == 0x02:  # TEACHING_MODE
            # Official 0x150 / drag-teach "end recording / exit teach".
            self.interface.MotionCtrl_1(0x00, 0x00, 0x02)
            if not self._wait_until_not_teaching():
                rospy.logerr("Leader did not leave TEACHING_MODE safely")
                return False
        self.interface.ModeCtrl(0x01, 0x01, self.initial_pose_speed, 0x00)
        if not self._wait_ctrl_mode(0x01):
            rospy.logerr("Leader did not enter CAN control mode")
            return False
        return True

    def _return_to_teaching_mode(self):
        # Official 0x150 / drag-teach "start recording / enter teach".
        # No disable_arm() is issued, so this path does not intentionally
        # release motor torque during the mode transition.
        self.interface.MotionCtrl_1(0x00, 0x00, 0x01)
        if not self._wait_ctrl_mode(0x02):
            rospy.logerr("Leader did not re-enter TEACHING_MODE")
            return False
        return True

    # =========================================================
    # 起動までTeacherモードを起動した際のサーボモータ初期化の関数たち
    # =========================================================
    def get_pos(self):
        '''Get the current joint radians of the robotic arm and the gripper opening distance'''
        joint_state = self.piper.get_joint_states()[0]
        if self.have_gripper:
            return joint_state + (self.piper.get_gripper_states()[0][0], )
        return joint_state 
    
    def stop(self):
        '''Stop the robotic arm; this function must be called first when exiting the teaching mode for the first time to control the robotic arm in CAN mode'''
        self.interface.EmergencyStop(0x01)
        time.sleep(1.0)
        limit_angle = [0.1745, 0.7854, 0.2094]  # The robotic arm can be restored only when the radians of joints 2, 3, and 5 are within the limit range to prevent damage caused by falling from a large radian
        pos = self.get_pos()
        while not (abs(pos[1]) < limit_angle[0] and abs(pos[2]) < limit_angle[0] and pos[4] < limit_angle[1] and pos[4] > limit_angle[2]):
            time.sleep(0.01)
            pos = self.get_pos()
        # Restore the robotic arm
        self.piper.disable_arm()
        time.sleep(1.0)
    
    def enable(self):
        '''Enable the robotic arm and gripper'''
        while not self.piper.enable_arm():
            time.sleep(0.01)
        if self.have_gripper:
            time.sleep(0.01)
            self.piper.enable_gripper()
        self.interface.ModeCtrl(0x01, 0x01, self.move_spd_rate_ctrl, 0x00)
        print("INFO: Enable successful")
        


# =============================================================
if __name__ == "__main__":
    node = PiperLeaderNode()
    rospy.spin()
