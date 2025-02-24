cmake_minimum_required(VERSION 2.8.3)
project(collision_avoidance_m100_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  #dji_sdk
  tf
)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hku_m100_gazebo
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  src/scripted_motion
	${catkin_INCLUDE_DIRS}
)

add_executable(m100_simulator_interface src/m100_simulator_interface.cpp)

add_library(scripted_motion
  src/scripted_motion/scripted_motion.cpp
  src/scripted_motion/scripted_motion_factory.cpp
  src/scripted_motion/scripted_motion_none.h
  src/scripted_motion/scripted_motion_circle.cpp
  src/scripted_motion/scripted_motion_path.cpp
)

add_executable(simulator_path_generation_nodelet 
  src/simulator_path_generation_nodelet.cpp
)

target_link_libraries(m100_simulator_interface
  ${catkin_LIBRARIES}
)

target_link_libraries(simulator_path_generation_nodelet
  ${catkin_LIBRARIES}
  scripted_motion
)

#############
## Install ##
#############

install(FILES
    launch/start.launch
    launch/spawn_m100.launch
    launch/start_m100_interface.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(FILES
    world/test.world
    world/test2.world
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/world
)

#############
## Testing ##
#############


