cmake_minimum_required(VERSION 3.0.2)
project(piper_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
add_message_files(FILES PiperStatusMsg.msg PosCmd.msg PiperEulerPose.msg)
add_service_files(FILES Enable.srv Gripper.srv GoZero.srv)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
