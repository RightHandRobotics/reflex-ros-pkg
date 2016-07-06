cmake_minimum_required(VERSION 2.8.3)
project(reflex)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  reflex_msgs
  rospy
  roscpp
  tf
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package()

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(tf_broadcaster src/tf_broadcaster.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})
add_dependencies(tf_broadcaster ${catkin_EXPORTED_TARGETS})
