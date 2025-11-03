cmake_minimum_required(VERSION 3.0.2)
project(scout_xarm_complete)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  control_msgs
  xacro
  robot_state_publisher
  gazebo_ros
  gazebo_ros_control
  controller_manager
  tf
)

## System dependencies
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    trajectory_msgs
    control_msgs
    xacro
    robot_state_publisher
    gazebo_ros
    controller_manager
    tf
)

###########
## Build ##
###########

## Install scripts
catkin_install_python(PROGRAMS
  scripts/xarm6_keyboard_teleop.py
  scripts/xarm6_init_home.py
  scripts/gripper_init_position.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

## Install URDF files
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  PATTERN ".svn" EXCLUDE
)

## Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)

## Install meshes
install(DIRECTORY meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
  PATTERN ".svn" EXCLUDE
)

## Install world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
  PATTERN ".svn" EXCLUDE
)

###########
## Test ##
###########

