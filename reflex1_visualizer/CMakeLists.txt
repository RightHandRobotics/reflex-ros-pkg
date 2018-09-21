cmake_minimum_required(VERSION 2.8.3)
project(reflex1_visualizer)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  reflex_one_msgs
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
add_executable(hand1_visualizer src/hand_visualizer.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(hand1_visualizer ${catkin_LIBRARIES})
add_dependencies(hand1_visualizer reflex_one_msgs_gencpp)
