cmake_minimum_required(VERSION 2.8.3)
project(lattice_planner)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  mav_msgs
  tf
  visualization_msgs
  message_generation
)

find_package(octomap REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++11 -O3 ")
#set(CMAKE_CXX_FLAGS "-std=c++11 -O3 -fsanitize=address -fsanitize=leak -fsanitize=undefined")
#set(CMAKE_CXX_FLAGS "-std=c++11 -O3 -g -fsanitize=address -fsanitize=leak -fsanitize=undefined")
#set(CMAKE_CXX_FLAGS "-std=c++11 -O3 -g -fsanitize=address -fsanitize=undefined")

add_message_files(
  FILES
  Plan.msg
  plantime.msg
  Obstacle.msg
  Obstacles.msg
  Scenario.msg
  BoundingBox.msg
  TrajectoryState.msg
)

generate_messages (
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
   nav_msgs
   mav_msgs
)

catkin_package(
  INCLUDE_DIRS include /usr/include/eigen3
  CATKIN_DEPENDS 
  roscpp 
  rospy 
  std_msgs 
  geometry_msgs 
  sensor_msgs 
  nav_msgs 
  mav_msgs
  tf
  catkin_simple
  visualization_msgs
  message_runtime
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
  ${OCTOMAP_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/latticePlanner_node.cpp
  src/LatticePlanner.cpp
)

add_dependencies(lattice_planner
lattice_planner_gencpp
${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(latticePlanner_node 
  src/latticePlanner_node.cpp
  src/LatticePlanner.cpp
  src/Obstacle.cpp
  src/PhysicalEnvironment.cpp
  src/Scenario.cpp
  src/UAV/MotionPrimitive.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(latticePlanner_node
   ${catkin_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   lattice_planner
)

install(FILES
    generate_primitives/primitives/primitive3.txt
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/primitives
)
