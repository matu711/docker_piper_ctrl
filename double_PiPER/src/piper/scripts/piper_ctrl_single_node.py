#!/usr/bin/env python3
# -*-coding:utf8-*-
# 本文件为控制单个机械臂节点，控制夹爪机械臂运动
from typing import (
    Optional,
)
import rospy
import rosnode
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse
import time
import threading
import argparse
import math
from piper_sdk import *
from piper_sdk import C_PiperInterface
from std_srvs.srv import Trigger, TriggerResponse
from piper_msgs.msg import PiperStatusMsg, PosCmd, PiperEulerPose
from piper_msgs.srv import Enable, EnableResponse
from piper_msgs.srv import Gripper, GripperResponse
from piper_msgs.srv import GoZero, GoZeroResponse
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_euler  # 用于欧拉角到四元数的转换
import numpy as np

def check_ros_master():
    try:
        rosnode.rosnode_ping('rosout', max_count=1, verbose=False)
        rospy.loginfo("ROS Master is running.")
    except rosnode.ROSNodeIOException:
        rospy.logerr("ROS Master is not running.")
        raise RuntimeError("ROS Master is not running.")

class C_PiperRosNode():
    """机械臂ros节点
    """
    def _create_piper_interface(self):
        self.start_sdk_joint_limit = rospy.get_param('~start_sdk_joint_limit', True)
        if type(self.start_sdk_joint_limit) is not bool:
            raise ValueError('~start_sdk_joint_limit must be a ROS boolean')
        rospy.loginfo("PiPER SDK can=%s start_sdk_joint_limit=%s",
                      self.can_port, self.start_sdk_joint_limit)
        return C_PiperInterface(can_name=self.can_port,
                                start_sdk_joint_limit=self.start_sdk_joint_limit)

    def __init__(self) -> None:
        check_ros_master()
        rospy.init_node('piper_ctrl_single_node', anonymous=True)
        # 外部param参数
        # can路由名称
        self.can_port = "can0"
        if rospy.has_param('~can_port'):
            self.can_port = rospy.get_param("~can_port")
            rospy.loginfo("%s is %s", rospy.resolve_name('~can_port'), self.can_port)
        else: 
            rospy.loginfo("未找到can_port参数")
            exit(0)
        # 是否自动使能，默认不自动使能
        self.auto_enable = False
        if rospy.has_param('~auto_enable'):
            if(rospy.get_param("~auto_enable")):
                self.auto_enable = True
        rospy.loginfo("%s is %s", rospy.resolve_name('~auto_enable'), self.auto_enable)
        # 是否有夹爪，默认为有
        self.gripper_exist = True
        if rospy.has_param('~gripper_exist'):
            if(not rospy.get_param("~gripper_exist")):
                self.gripper_exist = False
        rospy.loginfo("%s is %s", rospy.resolve_name('~gripper_exist'), self.gripper_exist)
        # 是否是打开了rviz控制，默认为不是，如果打开了，gripper订阅的joint7关节消息会乘2倍-------已弃用
        self.rviz_ctrl_flag = False
        if rospy.has_param('~rviz_ctrl_flag'):
            if(rospy.get_param("~rviz_ctrl_flag")):
                self.rviz_ctrl_flag = True
        rospy.loginfo("%s is %s", rospy.resolve_name('~rviz_ctrl_flag'), self.rviz_ctrl_flag)
        # 夹爪的数值倍数，默认为1
        self.gripper_val_mutiple = 1  # 默认值
        if rospy.has_param('~gripper_val_mutiple'):
            gripper_val_mutiple = rospy.get_param("~gripper_val_mutiple")
            # 检查是否为数字（浮动数或整数）
            if isinstance(gripper_val_mutiple, (int, float)):
                # 确保值在合理范围内
                if gripper_val_mutiple <= 0:
                    rospy.logwarn("Invalid gripper_val_mutiple value: must be positive. Using default value of 1.")
                    self.gripper_val_mutiple = 1  # 设置为默认值
                else:
                    self.gripper_val_mutiple = gripper_val_mutiple
            else:
                rospy.logwarn("Invalid gripper_val_mutiple type. Expected int or float. Using default value of 1.")
                self.gripper_val_mutiple = 1  # 设置为默认值
        else:
            rospy.logwarn("No gripper_val_mutiple param. Using default value of 1.")
            self.gripper_val_mutiple = 1  # 设置为默认值
        rospy.loginfo("%s is %s", rospy.resolve_name('~gripper_val_mutiple'), self.gripper_val_mutiple)
        # PiPER two-finger gripper opening range in metres.  The installed
        # gripper is specified and measured as 0--100 mm; keep this parameter
        # configurable for a different end effector while rejecting values
        # beyond the official 100 mm maximum.
        self.gripper_max_width_m = 0.10
        if rospy.has_param('~gripper_max_width_m'):
            gripper_max_width_m = rospy.get_param('~gripper_max_width_m')
            if isinstance(gripper_max_width_m, (int, float)) and 0 < gripper_max_width_m <= 0.10:
                self.gripper_max_width_m = float(gripper_max_width_m)
            else:
                rospy.logwarn(
                    'Invalid gripper_max_width_m %r; using 0.10 m.',
                    gripper_max_width_m,
                )
        self.gripper_max_command = round(self.gripper_max_width_m * 1000 * 1000)
        rospy.loginfo(
            '%s is %.3f m (%d SDK units)',
            rospy.resolve_name('~gripper_max_width_m'),
            self.gripper_max_width_m,
            self.gripper_max_command,
        )
        self.command_debug_stats = bool(
            rospy.get_param('~command_debug_stats', True)
        )
        self.command_diagnostic_period_s = float(
            rospy.get_param('~command_diagnostic_period_s', 1.0)
        )
        if self.command_diagnostic_period_s <= 0:
            rospy.logwarn(
                'command_diagnostic_period_s must be positive; using 1.0 s.'
            )
            self.command_diagnostic_period_s = 1.0
        # publish
        self.joint_pub = rospy.Publisher('joint_states_single', JointState, queue_size=1)
        self.arm_status_pub = rospy.Publisher('arm_status', PiperStatusMsg, queue_size=1)
        # self.end_pose_euler_pub = rospy.Publisher('end_pose_euler', PosCmd, queue_size=1)
        self.end_pose_pub = rospy.Publisher('end_pose', PoseStamped, queue_size=1)
        self.end_pose_euler_pub = rospy.Publisher('end_pose_euler', PiperEulerPose, queue_size=1)
        # service
        self.enable_service = rospy.Service('enable_srv', Enable, self.handle_enable_service)  # 创建enable服务
        self.__enable_flag = False
        self.gripper_service = rospy.Service('gripper_srv', Gripper, self.handle_gripper_service)  # 创建gripper服务
        self.stop_service = rospy.Service('stop_srv', Trigger, self.handle_stop_service)  # 创建stop服务
        self.reset_service = rospy.Service('reset_srv', Trigger, self.handle_reset_service)  # 创建reset服务
        self.go_zero_service = rospy.Service('go_zero_srv', GoZero, self.handle_go_zero_service)  # 创建reset服务
        self.block_arm_service = rospy.Service('block_arm', SetBool, self.handle_block_arm_service)
        # joint
        self.joint_states = JointState()
        self.joint_states.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        self.joint_states.position = [0.0] * 7
        self.joint_states.velocity = [0.0] * 7
        self.joint_states.effort = [0.0] * 7
        
        # 创建piper类并打开can接口
        self.piper = self._create_piper_interface()
        self.piper.ConnectPort()
        self.piper.MotionCtrl_2(0x01, 0x01, 30,0)
        self.block_ctrl_flag = False
        # 启动订阅线程
        sub_pos_th = threading.Thread(target=self.SubPosThread)
        sub_pos_th.daemon = True
        sub_pos_th.start()
        sub_joint_th = threading.Thread(target=self.SubJointThread)
        sub_enable_th = threading.Thread(target=self.SubEnableThread)
        
        sub_joint_th.daemon = True
        sub_enable_th.daemon = True
        
        sub_joint_th.start()
        sub_enable_th.start()

    def GetEnableFlag(self):
        return self.__enable_flag

    def Pubilsh(self):
        """机械臂消息发布
        """
        rate = rospy.Rate(200)  # 200 Hz
        enable_flag = False
        # 设置超时时间（秒）
        timeout = 5
        # 记录进入循环前的时间
        start_time = time.time()
        elapsed_time_flag = False
        while not rospy.is_shutdown():
            # print(self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status)
            if(self.auto_enable):
                while not (enable_flag):
                    elapsed_time = time.time() - start_time
                    print("--------------------")
                    enable_flag = self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
                    print("使能状态:",enable_flag)
                    self.piper.EnableArm(7)
                    self.piper.GripperCtrl(0,1000,0x01, 0)
                    if(enable_flag):
                        self.__enable_flag = True
                    print("--------------------")
                    # 检查是否超过超时时间
                    if elapsed_time > timeout:
                        print("超时....")
                        elapsed_time_flag = True
                        enable_flag = True
                        break
                    time.sleep(1)
                    pass
            if(elapsed_time_flag):
                print("程序自动使能超时,退出程序")
                exit(0)
            # 发布消息
            self.PublishArmState()
            self.PublishArmEndPose()
            self.PublishArmJointAndGripper()
            rate.sleep()

    def PublishArmState(self):
        # 机械臂状态
        arm_status = PiperStatusMsg()
        arm_status.ctrl_mode = self.piper.GetArmStatus().arm_status.ctrl_mode
        arm_status.arm_status = self.piper.GetArmStatus().arm_status.arm_status
        arm_status.mode_feedback = self.piper.GetArmStatus().arm_status.mode_feed
        arm_status.teach_status = self.piper.GetArmStatus().arm_status.teach_status
        arm_status.motion_status = self.piper.GetArmStatus().arm_status.motion_status
        arm_status.trajectory_num = self.piper.GetArmStatus().arm_status.trajectory_num
        arm_status.err_code = self.piper.GetArmStatus().arm_status.err_code
        arm_status.joint_1_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_1_angle_limit
        arm_status.joint_2_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_2_angle_limit
        arm_status.joint_3_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_3_angle_limit
        arm_status.joint_4_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_4_angle_limit
        arm_status.joint_5_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_5_angle_limit
        arm_status.joint_6_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_6_angle_limit
        arm_status.communication_status_joint_1 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_1
        arm_status.communication_status_joint_2 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_2
        arm_status.communication_status_joint_3 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_3
        arm_status.communication_status_joint_4 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_4
        arm_status.communication_status_joint_5 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_5
        arm_status.communication_status_joint_6 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_6
        self.arm_status_pub.publish(arm_status)
        
    def PublishArmJointAndGripper(self):
        # 机械臂关节角和夹爪位置
        # 由于获取的原始数据是度为单位扩大了1000倍，因此要转为弧度需要先除以1000，再乘3.14/180，然后限制小数点位数为5位
        joint_0:float = (self.piper.GetArmJointMsgs().joint_state.joint_1/1000) * 0.017444
        joint_1:float = (self.piper.GetArmJointMsgs().joint_state.joint_2/1000) * 0.017444
        joint_2:float = (self.piper.GetArmJointMsgs().joint_state.joint_3/1000) * 0.017444
        joint_3:float = (self.piper.GetArmJointMsgs().joint_state.joint_4/1000) * 0.017444
        joint_4:float = (self.piper.GetArmJointMsgs().joint_state.joint_5/1000) * 0.017444
        joint_5:float = (self.piper.GetArmJointMsgs().joint_state.joint_6/1000) * 0.017444
        joint_6:float = self.piper.GetArmGripperMsgs().gripper_state.grippers_angle/1000000
        vel_0:float = self.piper.GetArmHighSpdInfoMsgs().motor_1.motor_speed/1000
        vel_1:float = self.piper.GetArmHighSpdInfoMsgs().motor_2.motor_speed/1000
        vel_2:float = self.piper.GetArmHighSpdInfoMsgs().motor_3.motor_speed/1000
        vel_3:float = self.piper.GetArmHighSpdInfoMsgs().motor_4.motor_speed/1000
        vel_4:float = self.piper.GetArmHighSpdInfoMsgs().motor_5.motor_speed/1000
        vel_5:float = self.piper.GetArmHighSpdInfoMsgs().motor_6.motor_speed/1000
        effort_0:float = self.piper.GetArmHighSpdInfoMsgs().motor_1.effort/1000
        effort_1:float = self.piper.GetArmHighSpdInfoMsgs().motor_2.effort/1000
        effort_2:float = self.piper.GetArmHighSpdInfoMsgs().motor_3.effort/1000
        effort_3:float = self.piper.GetArmHighSpdInfoMsgs().motor_4.effort/1000
        effort_4:float = self.piper.GetArmHighSpdInfoMsgs().motor_5.effort/1000
        effort_5:float = self.piper.GetArmHighSpdInfoMsgs().motor_6.effort/1000
        effort_6:float = self.piper.GetArmGripperMsgs().gripper_state.grippers_effort/1000
        self.joint_states.header.stamp = rospy.Time.now()
        self.joint_states.position = [joint_0,joint_1, joint_2, joint_3, joint_4, joint_5,joint_6]
        self.joint_states.velocity = [vel_0, vel_1, vel_2, vel_3, vel_4, vel_5, 0.0]
        self.joint_states.effort = [effort_0, effort_1, effort_2, effort_3, effort_4, effort_5, effort_6]
        # 发布所有消息
        self.joint_pub.publish(self.joint_states)
    
    def PublishArmEndPose(self):
        # 末端位姿
        endpos = PoseStamped()
        endpos.pose.position.x = self.piper.GetArmEndPoseMsgs().end_pose.X_axis/1000000
        endpos.pose.position.y = self.piper.GetArmEndPoseMsgs().end_pose.Y_axis/1000000
        endpos.pose.position.z = self.piper.GetArmEndPoseMsgs().end_pose.Z_axis/1000000
        roll = self.piper.GetArmEndPoseMsgs().end_pose.RX_axis/1000
        pitch = self.piper.GetArmEndPoseMsgs().end_pose.RY_axis/1000
        yaw = self.piper.GetArmEndPoseMsgs().end_pose.RZ_axis/1000
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        endpos.pose.orientation.x = quaternion[0]
        endpos.pose.orientation.y = quaternion[1]
        endpos.pose.orientation.z = quaternion[2]
        endpos.pose.orientation.w = quaternion[3]
        endpos.header.stamp = rospy.Time.now()
        self.end_pose_pub.publish(endpos)
        
        end_pose_euler = PiperEulerPose()
        end_pose_euler.header.stamp = rospy.Time.now()
        # end_pose_euler.header.seq = endpos.header.seq
        end_pose_euler.x = self.piper.GetArmEndPoseMsgs().end_pose.X_axis/1000000
        end_pose_euler.y = self.piper.GetArmEndPoseMsgs().end_pose.Y_axis/1000000
        end_pose_euler.z = self.piper.GetArmEndPoseMsgs().end_pose.Z_axis/1000000
        end_pose_euler.roll = roll
        end_pose_euler.pitch = pitch
        end_pose_euler.yaw = yaw
        self.end_pose_euler_pub.publish(end_pose_euler)
    
    def SubPosThread(self):
        """机械臂末端位姿订阅
        
        """
        rospy.Subscriber('pos_cmd', PosCmd, self.pos_callback, queue_size=1, tcp_nodelay=True)
        rospy.spin()
    
    def SubJointThread(self):
        """机械臂关节订阅
        
        """
        rospy.Subscriber('joint_ctrl_single', JointState, self.joint_callback, queue_size=1, tcp_nodelay=True)
        # rospy.Subscriber('/move_group/fake_controller_joint_states', JointState, self.joint_callback)
        rospy.spin()
    
    def SubEnableThread(self):
        """机械臂使能
        
        """
        rospy.Subscriber('enable_flag', Bool, self.enable_callback, queue_size=1, tcp_nodelay=True)
        rospy.spin()

    def pos_callback(self, pos_data):
        """机械臂末端位姿订阅回调函数

        Args:
            pos_data (): 
        """
        period = self.command_diagnostic_period_s
        if self.block_ctrl_flag:
            rospy.logwarn_throttle(
                period,
                "Dropped /pos_cmd: block_ctrl_flag=true (block_arm service).",
            )
            return

        factor = 180 / 3.1415926
        x = round(pos_data.x*1000) * 1000
        y = round(pos_data.y*1000) * 1000
        z = round(pos_data.z*1000) * 1000
        rx = round(pos_data.roll*1000*factor)
        ry = round(pos_data.pitch*1000*factor)
        rz = round(pos_data.yaw*1000*factor)
        arm_status = self.piper.GetArmStatus().arm_status
        enabled = self.GetEnableFlag()
        if self.command_debug_stats:
            rospy.loginfo_throttle(
                period,
                "Received /pos_cmd: xyz_m=[%.6f, %.6f, %.6f] "
                "rpy_rad=[%.6f, %.6f, %.6f] gripper_m=%.6f "
                "msg_mode1=%d msg_mode2=%d enabled=%s blocked=%s "
                "arm_ctrl_mode=%s arm_status=%s mode_feedback=%s "
                "motion_status=%s err_code=%s SDK_xyz=[%d,%d,%d] "
                "SDK_rpy=[%d,%d,%d]",
                pos_data.x, pos_data.y, pos_data.z,
                pos_data.roll, pos_data.pitch, pos_data.yaw,
                pos_data.gripper, pos_data.mode1, pos_data.mode2,
                enabled, self.block_ctrl_flag, arm_status.ctrl_mode,
                arm_status.arm_status, arm_status.mode_feed,
                arm_status.motion_status, arm_status.err_code,
                x, y, z, rx, ry, rz,
            )
        if not enabled:
            low_speed = self.piper.GetArmLowSpdInfoMsgs()
            motor_enabled = [
                low_speed.motor_1.foc_status.driver_enable_status,
                low_speed.motor_2.foc_status.driver_enable_status,
                low_speed.motor_3.foc_status.driver_enable_status,
                low_speed.motor_4.foc_status.driver_enable_status,
                low_speed.motor_5.foc_status.driver_enable_status,
                low_speed.motor_6.foc_status.driver_enable_status,
            ]
            rospy.logwarn_throttle(
                period,
                "Dropped /pos_cmd: driver enable gate is false; "
                "motor_enable_status=%s arm_ctrl_mode=%s arm_status=%s "
                "err_code=%s",
                motor_enabled, arm_status.ctrl_mode, arm_status.arm_status,
                arm_status.err_code,
            )
            return

        # This is the same MOVE P sequence used by the official SDK example.
        # EndPoseCtrl performs no host-side IK: it transmits CAN 0x152/0x153/
        # 0x154 and the arm firmware performs planning/IK.
        self.piper.MotionCtrl_1(0x00, 0x00, 0x00)
        self.piper.MotionCtrl_2(0x01, 0x00, 50)
        self.piper.EndPoseCtrl(x, y, z, rx, ry, rz)
        gripper = round(pos_data.gripper*1000*1000)
        if(pos_data.gripper > self.gripper_max_width_m): gripper = self.gripper_max_command
        if(pos_data.gripper<0): gripper = 0
        if(self.gripper_exist):
            self.piper.GripperCtrl(abs(gripper), 1000, 0x01, 0)
        self.piper.MotionCtrl_2(0x01, 0x00, 50)
        if self.command_debug_stats:
            rospy.loginfo_throttle(
                period,
                "Invoked PiPER SDK for /pos_cmd: MotionCtrl_1(0,0,0), "
                "MotionCtrl_2(CAN,MOVE_P,50), EndPoseCtrl -> CAN "
                "[0x152,0x153,0x154]. SDK methods report CAN send failures "
                "through the piper_sdk logger; firmware IK result is visible "
                "in /arm_status arm_status/motion_status.",
            )
    
    def joint_callback(self, joint_data):
        """机械臂关节角回调函数

        Args:
            joint_data (): 
        """
        if not self.block_ctrl_flag:
            factor = 57324.840764 #1000*180/3.14
            factor = 1000 * 180 / np.pi
            # rospy.loginfo("Received Joint States:")
            # rospy.loginfo("joint_0: %f", joint_data.position[0])
            # rospy.loginfo("joint_1: %f", joint_data.position[1])
            # rospy.loginfo("joint_2: %f", joint_data.position[2])
            # rospy.loginfo("joint_3: %f", joint_data.position[3])
            # rospy.loginfo("joint_4: %f", joint_data.position[4])
            # rospy.loginfo("joint_5: %f", joint_data.position[5])
            # rospy.loginfo("joint_6: %f", joint_data.position[6])
            # print(joint_data.position)
            joint_0 = round(joint_data.position[0]*factor)
            joint_1 = round(joint_data.position[1]*factor)
            joint_2 = round(joint_data.position[2]*factor)
            joint_3 = round(joint_data.position[3]*factor)
            joint_4 = round(joint_data.position[4]*factor)
            joint_5 = round(joint_data.position[5]*factor)
            if(len(joint_data.position) >= 7):
                joint_6 = round(joint_data.position[6]*1000*1000)
                joint_6 = joint_6 * self.gripper_val_mutiple
                if(joint_6 > self.gripper_max_command): joint_6 = self.gripper_max_command
                if(joint_6<0): joint_6 = 0
            else: joint_6 = None
            if(self.GetEnableFlag()):
                # 设定电机速度
                if(joint_data.velocity != []):
                    all_zeros = all(v == 0 for v in joint_data.velocity)
                else: all_zeros = True
                if(not all_zeros):
                    lens = len(joint_data.velocity)
                    if(lens == 7):
                        vel_all = round(joint_data.velocity[6])
                        if (vel_all > 100): vel_all = 100
                        if (vel_all < 0): vel_all = 0
                        rospy.loginfo("vel_all: %d", vel_all)
                        self.piper.MotionCtrl_2(0x01, 0x01, vel_all)
                    # elif(lens == 7):
                    #     # 遍历速度列表
                    #     for i, velocity in enumerate(joint_data.velocity):
                    #         if velocity > 0:  # 如果速度是正数
                    #             # 设置指定位置的关节速度为这个正数速度
                    #             # self.piper.SearchMotorMaxAngleSpdAccLimit(i+1,0x01)
                    #             # self.piper.MotorAngleLimitMaxSpdSet(i+1)
                    else: self.piper.MotionCtrl_2(0x01, 0x01, 50,0)
                else: self.piper.MotionCtrl_2(0x01, 0x01, 50,0)
                
                # 给定关节角位置
                self.piper.JointCtrl(joint_0, joint_1, joint_2, 
                                        joint_3, joint_4, joint_5)
                # 如果末端夹爪存在，则发送末端夹爪控制
                if(self.gripper_exist and joint_6 is not None):
                    if abs(joint_6)<200:
                        joint_6=0
                    if(len(joint_data.effort) >= 7):
                        gripper_effort = joint_data.effort[6]
                        gripper_effort = max(0.5, min(gripper_effort, 3))
                        # rospy.loginfo("gripper_effort: %f", gripper_effort)
                        gripper_effort = round(gripper_effort*1000)
                        self.piper.GripperCtrl(abs(joint_6), gripper_effort, 0x01, 0)
                    # 默认1N
                    else: self.piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)
    
    def enable_callback(self, enable_flag:Bool):
        """机械臂使能回调函数

        Args:
            enable_flag (): 
        """
        rospy.loginfo("Received enable flag:")
        rospy.loginfo("enable_flag: %s", enable_flag.data)
        if(enable_flag.data):
            self.__enable_flag = True
            self.piper.EnableArm(7)
            if(self.gripper_exist):
                self.piper.GripperCtrl(0,1000,0x01, 0)
        else:
            self.__enable_flag = False
            self.piper.DisableArm(7)
            if(self.gripper_exist):
                self.piper.GripperCtrl(0,1000,0x00, 0)
    
    def handle_gripper_service(self,req):
        response = GripperResponse()
        response.code = 15999
        response.status = False
        if(self.gripper_exist):
            rospy.loginfo(f"-----------------------Gripper---------------------------")
            rospy.loginfo(f"Received request:")
            rospy.loginfo(f"PS: Piper should be enable.Please ensure piper is enable")
            rospy.loginfo(f"gripper_angle:{req.gripper_angle}, range is [0m, 0.07m]")
            rospy.loginfo(f"gripper_effort:{req.gripper_effort},range is [0.5N/m, 2N/m]")
            rospy.loginfo(f"gripper_code:{req.gripper_code}, range is [0, 1, 2, 3]\n \
                            0x00: Disable\n \
                            0x01: Enable\n \
                            0x03/0x02: Enable and clear error / Disable and clear error")
            rospy.loginfo(f"set_zero:{req.set_zero}, range is [0, 0xAE] \n \
                            0x00: Invalid value \n \
                            0xAE: Set zero point")
            rospy.loginfo(f"-----------------------Gripper---------------------------")
            gripper_angle = req.gripper_angle
            gripper_angle = round(max(0, min(req.gripper_angle, 0.07)) * 1e6)
            gripper_effort = req.gripper_effort
            gripper_effort = round(max(0.5, min(req.gripper_effort, 2)) * 1e3)
            if req.gripper_code not in [0x00, 0x01, 0x02, 0x03]:
                rospy.logwarn("gripper_code should be in [0, 1, 2, 3], default val is 1")
                gripper_code = 1
                response.code = 15901
            else: gripper_code = req.gripper_code
            if req.set_zero not in [0x00, 0xAE]:
                rospy.logwarn("set_zero should be in [0, 0xAE], default val is 0")
                set_zero = 0
                response.code = 15902
            else: set_zero = req.set_zero
            response.code = 15900
            self.piper.GripperCtrl(abs(gripper_angle), gripper_effort, gripper_code, set_zero)
            response.status = True
        else:
            rospy.logwarn("gripper_exist param is False.")
            response.code = 15903
            response.status = False
        rospy.loginfo(f"Returning GripperResponse: {response.code}, {response.status}")
        return response
    
    def handle_enable_service(self,req):
        rospy.loginfo(f"Received request: {req.enable_request}")
        enable_flag = False
        loop_flag = False
        # 设置超时时间（秒）
        timeout = 5
        # 记录进入循环前的时间
        start_time = time.time()
        elapsed_time_flag = False
        while not (loop_flag):
            elapsed_time = time.time() - start_time
            print("--------------------")
            enable_list = []
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status)
            if(req.enable_request):
                enable_flag = all(enable_list)
                self.piper.EnableArm(7)
                self.piper.GripperCtrl(0,1000,0x01, 0)
            else:
                enable_flag = any(enable_list)
                self.piper.DisableArm(7)
                self.piper.GripperCtrl(0,1000,0x02, 0)
            print("使能状态:", enable_flag)
            self.__enable_flag = enable_flag
            print("--------------------")
            if(enable_flag == req.enable_request):
                loop_flag = True
                enable_flag = True
            else: 
                loop_flag = False
                enable_flag = False
            # 检查是否超过超时时间
            if elapsed_time > timeout:
                print("超时....")
                elapsed_time_flag = True
                enable_flag = False
                loop_flag = True
                break
            time.sleep(0.5)
        response = enable_flag
        rospy.loginfo(f"Returning response: {response}")
        return EnableResponse(response)

    def handle_stop_service(self,req):
        response = TriggerResponse()
        response.success = False
        response.message = "stop piper failed"
        rospy.loginfo(f"-----------------------STOP---------------------------")
        rospy.loginfo(f"Stop piper.")
        rospy.loginfo(f"-----------------------STOP---------------------------")
        self.piper.MotionCtrl_1(0x01,0,0)
        response.success = True
        response.message = "stop piper success"
        rospy.loginfo(f"Returning StopResponse: {response.success}, {response.message}")
        return response

    def handle_reset_service(self,req):
        response = TriggerResponse()
        response.success = False
        response.message = "reset piper failed"
        rospy.loginfo(f"-----------------------RESET---------------------------")
        rospy.loginfo(f"reset piper.")
        rospy.loginfo(f"-----------------------RESET---------------------------")
        self.piper.MotionCtrl_1(0x02,0,0)#恢复
        response.success = True
        response.message = "reset piper success"
        rospy.loginfo(f"Returning resetResponse: {response.success}, {response.message}")
        return response

    def handle_go_zero_service(self,req):
        response = GoZeroResponse()
        response.status = False
        response.code = 151000
        rospy.loginfo(f"-----------------------GOZERO---------------------------")
        rospy.loginfo(f"piper go zero .")
        rospy.loginfo(f"-----------------------GOZERO---------------------------")
        if(req.is_mit_mode):
            self.piper.MotionCtrl_2(0x01, 0x01, 50, 0xAD)
        else:
            self.piper.MotionCtrl_2(0x01, 0x01, 50, 0)
        self.piper.JointCtrl(0, 0, 0, 0, 0, 0)
        response.status = True
        response.code = 151001
        rospy.loginfo(f"Returning GoZeroResponse: {response.status}, {response.code}")
        return response

    def handle_block_arm_service(self,req):
        response = SetBoolResponse()
        rospy.loginfo(f"-----------------------BLOCK_ARM---------------------------")
        if req.data:
            response.success = req.data
            response.message = "You will block arm ctrl msg send"
        else:
            response.success = req.data
            response.message = "You will unblock arm ctrl msg send"
        self.block_ctrl_flag = req.data
        rospy.loginfo(f"piper block arm .")
        rospy.loginfo(f"Returning BlockArmResponse: {response.success}, {response.message}")
        rospy.loginfo(f"-----------------------BLOCK_ARM---------------------------")
        return response
    
if __name__ == '__main__':
    try:
        piper_signle = C_PiperRosNode()
        piper_signle.Pubilsh()
    except rospy.ROSInterruptException:
        pass
