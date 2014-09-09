cmake_minimum_required(VERSION 2.8.3)
project(reflex_driver)
find_package(catkin REQUIRED COMPONENTS roscpp)
catkin_package( CATKIN_DEPENDS roscpp)

add_library(reflex_hand reflex_hand.cpp)

add_executable(reflex_driver_node reflex_driver_node.cpp)
target_link_libraries(reflex_driver_node reflex_hand ${catkin_LIBRARIES})

add_executable(dump_state_to_console dump_state_to_console.cpp)
target_link_libraries(dump_state_to_console reflex_hand ${catkin_LIBRARIES})
