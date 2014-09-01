cmake_minimum_required(VERSION 2.8.3)
project(reflex_base)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  reflex_msgs
  rospy
  roscpp
  tf
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES reflex_base
#  CATKIN_DEPENDS reflex_msgs rospy takktile_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(reflex_base
#   src/${PROJECT_NAME}/reflex_base.cpp
# )

## Declare a cpp executable
add_executable(reflex_tf_broadcaster src/reflex_tf_broadcaster.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(reflex_base_node reflex_base_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(reflex_tf_broadcaster ${catkin_LIBRARIES})