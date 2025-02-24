cmake_minimum_required(VERSION 2.8.3)
project(laser_to_pcl)

find_package(catkin REQUIRED COMPONENTS
  rospy
  pysdf
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES laser_to_pcl
  CATKIN_DEPENDS rospy pysdf
  DEPENDS python-rospkg
)


###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############
install(PROGRAMS
  src/laser_to_pcl/laser2pc.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
