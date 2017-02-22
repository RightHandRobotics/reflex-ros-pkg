cmake_minimum_required(VERSION 2.8.3)
project(reflex_driver)
find_package(catkin REQUIRED COMPONENTS roscpp reflex_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})
catkin_package( CATKIN_DEPENDS roscpp reflex_msgs)

add_library(reflex_hand reflex_hand.cpp)
add_library(reflex_driver_lib reflex_driver.cpp)

add_executable(hand1_reflex_driver_node hand1_reflex_driver_node.cpp)
add_executable(hand2_reflex_driver_node hand2_reflex_driver_node.cpp)
target_link_libraries(hand1_reflex_driver_node reflex_hand reflex_driver_lib ${catkin_LIBRARIES})
target_link_libraries(hand2_reflex_driver_node reflex_hand reflex_driver_lib ${catkin_LIBRARIES})
add_dependencies(reflex_driver_lib std_srvs_gencpp reflex_msgs_gencpp)