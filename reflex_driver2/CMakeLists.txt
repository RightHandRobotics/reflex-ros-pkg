cmake_minimum_required(VERSION 2.8.3)
project(reflex_driver2)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp)

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

##
catkin_package(CATKIN_DEPENDS roscpp reflex_msgs)

##
add_library(reflex_hand2 reflex_hand.cpp)

## Declare a cpp executable
add_executable(reflex_driver_node2 reflex_driver_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(reflex_driver_node2 reflex_hand ${catkin_LIBRARIES})
add_dependencies(reflex_driver_node2 std_srvs_gencpp reflex_msgs_gencpp)
