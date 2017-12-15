cmake_minimum_required(VERSION 2.8.3)
project(reflex_driver)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp reflex_msgs)

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

##
catkin_package(CATKIN_DEPENDS roscpp reflex_msgs)

##
add_library(reflex_hand src/reflex_hand.cpp)

## Declare a cpp executable
add_executable(reflex_driver_node src/reflex_driver_node.cpp)
add_executable(dump_state_to_console dump_state_to_console.cpp ${CATKIN_LIBRARIES})

## Specify libraries to link a library or executable target against
target_link_libraries(reflex_driver_node reflex_hand ${catkin_LIBRARIES})
target_link_libraries(dump_state_to_console reflex_hand ${catkin_LIBRARIES})

add_dependencies(reflex_driver_node std_srvs_gencpp reflex_msgs_gencpp)
