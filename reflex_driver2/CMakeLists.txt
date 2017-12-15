cmake_minimum_required(VERSION 2.8.3)
project(reflex_driver2)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp reflex_msgs2) # ORIGINAL
# find_package(catkin REQUIRED COMPONENTS roscpp) #makes
# find_package(catkin REQUIRED COMPONENTS reflex_msgs2) # does not compile

## Specify additional locations of header files
#makes if commented out
include_directories(include ${catkin_INCLUDE_DIRS}) 

#makes if commented out
catkin_package(CATKIN_DEPENDS roscpp reflex_msgs2)

#breaks it commented out
add_library(reflex_hand2 src/reflex_hand.cpp)

## Declare a cpp executable NODE DECLARATION. something something target
# MUST be named reflex_driver_node2
#breaks if commented out
add_executable(reflex_driver_node2 src/reflex_driver_node.cpp) 

## Specify libraries to link a library or executable target against
#breaks if commented out
target_link_libraries(reflex_driver_node2 reflex_hand2 ${catkin_LIBRARIES})

#makes if commented out
add_dependencies(reflex_driver_node2 std_srvs_gencpp reflex_msgs2_gencpp) 
