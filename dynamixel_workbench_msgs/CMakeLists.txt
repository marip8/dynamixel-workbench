################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_workbench_msgs)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  genmsg
  message_generation
  std_msgs
  sensor_msgs
  actionlib_msgs
)

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_message_files(
  FILES
  DynamixelAX.msg
  DynamixelRX.msg
  DynamixelMX.msg
  DynamixelMX64.msg
  DynamixelMX106.msg
  DynamixelEX.msg
  DynamixelXL.msg
  DynamixelXL320.msg
  DynamixelXM.msg
  DynamixelXH.msg
  DynamixelPro.msg
  DynamixelProL42.msg
  MotorState.msg
  MotorStateList.msg
  DynamixelCommand.msg
  WorkbenchParam.msg
)

add_service_files(
  FILES
  SetPosition.srv
  SetDirection.srv
  GetWorkbenchParam.srv
  MoveToPosition.srv
)

add_action_files(
    DIRECTORY action 
    FILES MoveToPosition.action)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  actionlib_msgs
)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
   CATKIN_DEPENDS std_msgs genmsg message_runtime
)

################################################################################
# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
)

################################################################################
# Install
################################################################################


################################################################################
# Test
################################################################################
