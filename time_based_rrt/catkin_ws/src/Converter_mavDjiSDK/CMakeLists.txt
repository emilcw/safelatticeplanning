cmake_minimum_required(VERSION 2.8.3)
project(converter_mav_dji_sdk)

set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   geometry_msgs
   sensor_msgs
   nav_msgs
)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++11")

catkin_package(
   INCLUDE_DIRS include /usr/include/eigen3
#  LIBRARIES converter_djisdk_mav
#  DEPENDS system_lib
   CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs nav_msgs catkin_simple
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  /usr/include/eigen3
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(converter_djisdk_mav_node src/converter_djisdk_mav_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(converter_djisdk_mav_node
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
 install(PROGRAMS
   scripts/convert_mavctrl_to_DJISDK_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
