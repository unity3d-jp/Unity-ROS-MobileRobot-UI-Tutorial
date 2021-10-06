cmake_minimum_required(VERSION 3.0.2)
project(raspimouse_msgs)

set(MESSAGE_DEPENDS
  actionlib_msgs
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS
  message_generation ${MESSAGE_DEPENDS}
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  LightSensorValues.msg
  LedValues.msg
  ButtonValues.msg
  MotorFreqs.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  TimedMotion.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  Music.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  ${MESSAGE_DEPENDS}
)

###################################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS
  message_runtime
  ${MESSAGE_DEPENDS}
)
