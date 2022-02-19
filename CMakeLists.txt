#
# Project settings
#

# Minimum required version of cmake
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

# Project name and programming languages used
PROJECT( lattice_acado CXX )

# CMake module(s) path
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

#
# Prerequisites
#
FIND_PACKAGE( ACADO REQUIRED )

#
# Include directories
#
INCLUDE_DIRECTORIES( include ${ACADO_INCLUDE_DIRS} )

# Build an executable
#
ADD_EXECUTABLE(        simple src/simple_ocp.cpp)
# Linux and OS X
TARGET_LINK_LIBRARIES( simple ${ACADO_SHARED_LIBRARIES} )
# Windows
#TARGET_LINK_LIBRARIES( <EXEC_NAME> ${ACADO_STATIC_LIBRARIES} )
SET_TARGET_PROPERTIES( simple
                       PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )


ADD_LIBRARY(acado_math_functions src/acado_math/acado_math_functions.cpp)


ADD_EXECUTABLE(        mprim src/mprim.cpp)
# Linux and OS X
TARGET_LINK_LIBRARIES( mprim acado_math_functions ${ACADO_SHARED_LIBRARIES} )

SET_TARGET_PROPERTIES( mprim
                       PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )
