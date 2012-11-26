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

# Path to SKYAI_BASE_DIR/materials/cmake
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../materials/cmake)
include(UseSkyAI)


use_skyai(skyai)
include_directories(${SKYAI_INCLUDE_DIRS})
link_directories (${SKYAI_LIBRARY_DIRS})
add_executable (maze.out maze.cpp)
target_link_libraries (maze.out ${SKYAI_LIBRARIES})
