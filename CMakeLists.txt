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
set(SKYAI_VERSION "0.3.0")
set(SKYAI_SOVERSION "1")

string(REGEX MATCHALL "[0-9]" SKYAI_VERSION_PARTS "${SKYAI_VERSION}")

list(GET SKYAI_VERSION_PARTS 0 SKYAI_VERSION_MAJOR)
list(GET SKYAI_VERSION_PARTS 1 SKYAI_VERSION_MINOR)
list(GET SKYAI_VERSION_PARTS 2 SKYAI_VERSION_PATCH)
##-------------------------------------------------------------------------------------------
set (BUILD_3RDPARTY ON CACHE BOOL "Build 3rdparty")
set (BUILD_LORA ON CACHE BOOL "Build liblora")
set (BUILD_SKYAI ON CACHE BOOL "Build libskyai")
set (BUILD_BENCHMARKS ON CACHE BOOL "Build benchmarks")
set (BUILD_TOOLS ON CACHE BOOL "Build tools")
set (BUILD_TEST OFF CACHE BOOL "Build test")
##-------------------------------------------------------------------------------------------
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib/skyai)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib/skyai)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin/skyai)
##-------------------------------------------------------------------------------------------
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/materials/cmake)
##-------------------------------------------------------------------------------------------
include(UseSkyAI)

if (BUILD_3RDPARTY)
  add_subdirectory (3rdparty)
endif()
if (BUILD_LORA)
  add_subdirectory (liblora)
endif()
if (BUILD_SKYAI)
  add_subdirectory (libskyai)
  set(SKYAI_INSTALL_PREFIX ${PROJECT_BINARY_DIR})
  configure_file(default.agent.in ${PROJECT_BINARY_DIR}/bin/skyai/default.agent)
  set(SKYAI_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
  configure_file(default.agent.in ${PROJECT_BINARY_DIR}/default.agent)
  install (FILES ${PROJECT_BINARY_DIR}/default.agent DESTINATION bin/skyai)
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
##-------------------------------------------------------------------------------------------
execute_process (
    COMMAND mkdir -p share/skyai
    COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/materials share/skyai/
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
  )
install (DIRECTORY materials/ DESTINATION share/skyai/materials
    FILES_MATCHING
      PATTERN "*.cmake"
      PATTERN "*.pdf"
      PATTERN "*.svg"
      PATTERN "*.png"
      PATTERN "*.cpp"
      PATTERN "*.h"
      PATTERN "*.xml"
      PATTERN "*.var"
      PATTERN "*.py")
##-------------------------------------------------------------------------------------------
