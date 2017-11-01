cmake_minimum_required(VERSION 2.8.3)
project(reflex_visualizer2)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  reflex
  reflex_msgs2
  roscpp
  sensor_msgs
  urdf
  visualization_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(hand_visualizer2 src/hand_visualizer.cpp)
#add_executable(reflex imu.py)

## Specify libraries to link a library or executable target against
target_link_libraries(hand_visualizer2 ${catkin_LIBRARIES})
add_dependencies(hand_visualizer2 reflex_msgs2_gencpp)
