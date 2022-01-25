cmake_minimum_required(VERSION 2.8.3)
project(object_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  water_msgs
)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})

#To solve the error which gazebo does not handle the include file well, we must add this line.
include_directories(/usr/include/gazebo-7/gazebo)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
         include
        ${catkin_INCLUDE_DIRS}
)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS tf
)

# Python Scripts
catkin_install_python(PROGRAMS
  scripts/generate_move.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

