cmake_minimum_required(VERSION 2.8.3)
project(reflex_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  std_srvs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Hand.msg
  Palm.msg
  Finger.msg
  Dynamixel.msg
  RawServoPositions.msg
  RadianServoPositions.msg
  MotorDebug.msg
  StateDebug.msg
  TrackState.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  CommandHand.srv
  MoveFinger.srv
  MovePreshape.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 CATKIN_DEPENDS std_msgs std_srvs message_runtime
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)
