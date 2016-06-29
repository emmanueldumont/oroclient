cmake_minimum_required(VERSION 2.8.3)
project(oroclient)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()
install(DIRECTORY include/${orolib}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)

include_directories(${catkin_INCLUDE_DIRS})
#include_directories(${orolib_INCLUDE_DIRS})

# Add executable present in "src"
add_subdirectory(src)
