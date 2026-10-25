cmake_minimum_required(VERSION 3.0.2)
project(piper)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  piper_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
)

catkin_package(CATKIN_DEPENDS
  cv_bridge geometry_msgs piper_msgs rospy sensor_msgs std_msgs std_srvs tf
)

# Preserve local imports and default ../config paths in source and install space.
install(DIRECTORY scripts/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.py" PATTERN "__pycache__" EXCLUDE
)
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
