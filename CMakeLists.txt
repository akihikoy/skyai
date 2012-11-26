##-------------------------------------------------------------------------------------------
cmake_minimum_required (VERSION 2.4)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

if(DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of
build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug
Release RelWithDebInfo MinSizeRel.")
else()
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build,
options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release
RelWithDebInfo MinSizeRel.")
endif()

# add the shared library path to rpath on the executables:
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/skyai")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
##-------------------------------------------------------------------------------------------
project (SKYAI)
##-------------------------------------------------------------------------------------------
set(SKYAI_VERSION "0.2.0")

string(REGEX MATCHALL "[0-9]" SKYAI_VERSION_PARTS "${SKYAI_VERSION}")

list(GET SKYAI_VERSION_PARTS 0 SKYAI_VERSION_MAJOR)
list(GET SKYAI_VERSION_PARTS 1 SKYAI_VERSION_MINOR)
list(GET SKYAI_VERSION_PARTS 2 SKYAI_VERSION_PATCH)

set(SKYAI_SOVERSION "${SKYAI_VERSION_MAJOR}.${SKYAI_VERSION_MINOR}")
##-------------------------------------------------------------------------------------------
set (BUILD_3RDPARTY ON CACHE BOOL "Build 3rdparty")
set (BUILD_LORA ON CACHE BOOL "Build liblora")
set (BUILD_SKYAI ON CACHE BOOL "Build libskyai")
set (BUILD_BENCHMARKS ON CACHE BOOL "Build benchmarks")
set (BUILD_TOOLS ON CACHE BOOL "Build tools")
set (BUILD_TEST OFF CACHE BOOL "Build test")


list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/materials/cmake)
include(UseSkyAI)


if (BUILD_3RDPARTY)
  add_subdirectory (3rdparty)
endif()
if (BUILD_LORA)
  add_subdirectory (liblora)
endif()
if (BUILD_SKYAI)
  add_subdirectory (libskyai)
endif()
if (BUILD_BENCHMARKS)
  add_subdirectory (benchmarks)
endif()
if (BUILD_TOOLS)
  add_subdirectory (tools)
endif()
if (BUILD_TEST)
  add_subdirectory (test)
endif()

