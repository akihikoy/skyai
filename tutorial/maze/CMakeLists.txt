cmake_minimum_required (VERSION 2.4)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# Set default build type as Release
if(DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Build type.")
else()
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type.")
endif()

project (MAZE)

# Path to SkyAI's build directory or install directory
set (SKYAI_INSTALL_DIR "${PROJECT_SOURCE_DIR}/../../build" CACHE PATH "Path to the SkyAI's build directory or install directory")

# Not use ODE and OpenCV
set (WITH_ODE OFF CACHE BOOL "")
set (WITH_OPENCV OFF CACHE BOOL "")

list(APPEND CMAKE_MODULE_PATH ${SKYAI_INSTALL_DIR}/share/skyai/materials/cmake)
include(UseSkyAI)

# Set library output directory to the source directory
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

use_skyai(skyai)
include_directories(${SKYAI_INCLUDE_DIRS})
link_directories (${SKYAI_LIBRARY_DIRS})
add_library (maze SHARED maze.cpp)
target_link_libraries (maze ${SKYAI_LIBRARIES})
