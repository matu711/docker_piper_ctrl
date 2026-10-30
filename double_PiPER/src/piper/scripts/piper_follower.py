#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math
import rospy
from sensor_msgs.msg import JointState
from piper_sdk import Piper



class PiperFollowerNode:

    def __init__(self):

        rospy.init_node("piper_ctrl_follower_node")

        # =========================
        # params
        # =========================
        self.can_port = rospy.get_param("~can_port", "can0")
        self.auto_enable = rospy.get_param("~auto_enable", True)
        self.gripper_exist = rospy.get_param("~gripper_exist", True)

        self.move_spd_rate_ctrl = 40
        self.have_gripper = True

        # 最新のleader姿勢保存用
        self.latest_pose = None

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

        ### -----

        if self.auto_enable:
            while not self.piper.enable_arm():
                time.sleep(0.01)

            if self.gripper_exist:
                self.piper.enable_gripper()

        self.interface.ModeCtrl(0x01, 0x01, self.move_spd_rate_ctrl, 0x00)

        rospy.loginfo("Follower %s ready.", self.can_port)

        # Asynchronous publication adds no recorder/file work to control.
        self.applied_cmd_pub = rospy.Publisher(
            "applied_joint_cmd", JointState, queue_size=10
        )

        # =========================
        # Subscribers
        # =========================
        rospy.Subscriber(
            "joint_ctrl_single",
            JointState,
            self.joint_callback,
            queue_size=1
        )

        # rospy.Subscriber(
        #     "/end_pose_euler",
        #     PiperEulerPose,
        #     self.pose_callback,
        #     queue_size=1
        # )
        rospy.Subscriber(
            "/end_pose_euler",
            JointState,
            self.pose_callback,
            queue_size=1
        )

        self.joint_obs_pub = rospy.Publisher(
            "current_joint_obs",
            JointState,
            queue_size=10
        )
        self.robot_obs = JointState()
        self.robot_obs.name = [
            "joint1","joint2","joint3",
            "joint4","joint5","joint6",
            "gripper"
        ]
        # 定期publish用timer
        rospy.Timer(rospy.Duration(0.02), self.Publish_JointObs)

    # =========================================================
    # PiPERの現在のJoint角度＆グリッパー位置を取得
    # =========================================================
    def Publish_JointObs(self, event):
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
        self.joint_obs_pub.publish(msg)



    # =========================================================
    # leaderのエンド姿勢を保存
    # =========================================================
    def pose_callback(self, msg):
        self.latest_pose = msg
        # print("===== Leader End Pose =====")
        # print("x: {:.3f}  y: {:.3f}  z: {:.3f}".format(msg.x, msg.y, msg.z))
        # print("roll: {:.3f}  pitch: {:.3f}  yaw: {:.3f}".format(
        #     msg.roll, msg.pitch, msg.yaw))
        # print("===========================")

    # =========================================================
    # Leader追従
    # =========================================================
    def joint_callback(self, msg):

        # leader姿勢がまだ来ていない場合は動かさない
        if self.latest_pose is None:
            return

        if len(msg.position) < 7:
            return

        joints = list(msg.position[:6])
        gripper = msg.position[6]

        # gripper safety clamp
        gripper = max(0.0, min(0.1, gripper))

        # Record only calls eligible for SDK dispatch; do not change motion.
        publish_applied = self._can_publish_action()

        # ======= 動作実行 =======
        self.piper.move_j(joints, self.move_spd_rate_ctrl)

        if self.gripper_exist:
            self.piper.move_gripper(gripper, 1)

        if publish_applied and self.gripper_exist:
            self._publish_applied_command(joints, gripper)

        ## debug
        # print("=======================")
        # print(self.piper.get_end_pose_euler()[0])
        # print("=======================")

    def _can_publish_action(self):
        """Mirror SDK move_j eligibility without changing its dispatch path."""
        try:
            if not self.piper.get_ctrl_output_status():
                return False
            if self.interface.get_connect_status():
                return (all(self.interface.GetArmEnableStatus()) and
                        int(self.interface.GetArmStatus().arm_status.ctrl_mode) != 2)
            return True
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "Action eligibility unavailable: %s" % exc)
            return False

    def _publish_applied_command(self, joints, gripper):
        try:
            applied = JointState()
            applied.header.stamp = rospy.Time.now()
            applied.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]
            # Mirror installed SDK wire quantization; no change to SDK inputs.
            applied.position = [math.radians(round(math.degrees(j)*1000)/1000)
                                for j in joints] + [round(gripper*1e6)/1e6]
            self.applied_cmd_pub.publish(applied)
        except Exception as exc:
            # Recording availability must not affect established control.
            rospy.logwarn_throttle(1.0, "Action publication failed: %s" % exc)

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
    node = PiperFollowerNode()
    rospy.spin()
