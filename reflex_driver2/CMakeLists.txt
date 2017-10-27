cmake_minimum_required(VERSION 2.8.3)
project(reflex_driver2)
find_package(catkin REQUIRED COMPONENTS roscpp)
include_directories(include ${catkin_INCLUDE_DIRS})
catkin_package( CATKIN_DEPENDS roscpp reflex_msgs)

add_library(reflex_hand2 reflex_hand.cpp)

add_executable(reflex_driver_node2 reflex_driver_node.cpp)
target_link_libraries(reflex_driver_node2 reflex_hand ${catkin_LIBRARIES})
add_dependencies(reflex_driver_node2 std_srvs_gencpp reflex_msgs_gencpp)
