CMAKE_MINIMUM_REQUIRED(VERSION 2.8.4)
PROJECT(djioptimize)

# CMake module(s) path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

#
# Prerequisites
#
FIND_PACKAGE(ACADO REQUIRED)

#
# Include directories
#
INCLUDE_DIRECTORIES( 
	include
	${ACADO_INCLUDE_DIRS} 
)

set (CMAKE_CXX_FLAGS  "-std=c++11 -g")

#QT creator needs to find the included files..
set (PROJECT_INCLUDE
    include/MotionPrimitiveGeneration.h
    include/Trajectory.h
    include/State.h
    include/ACADODjiMatrice.h
    include/ACADODjiMatriceHelper.h
    include/GridPoint.h
    include/State.h
)

set (PROJECT_SRC
    src/MotionPrimitiveGeneration.cpp
    src/Trajectory.cpp
    src/State.cpp
    src/ACADODjiMatrice.cpp
    src/ACADODjiMatriceHelper.cpp
    src/GridPoint.cpp
)

#
# Build an executable
#
ADD_EXECUTABLE(djioptimize_run src/main.cpp ${PROJECT_SRC} ${PROJECT_INCLUDE})

TARGET_LINK_LIBRARIES(djioptimize_run 
${ACADO_SHARED_LIBRARIES})







