# CMakeLists.txt for test codes

# usage: add_skyai_executable(file_prefix [extra_libs])
function(add_skyai_executable file_prefix)
  add_executable (${file_prefix}.out ${file_prefix}.cpp)
  target_link_libraries (${file_prefix}.out ${ARGN} ${SKYAI_LIBRARIES})
endfunction()

execute_process (COMMAND
    ln -sf ${CMAKE_CURRENT_SOURCE_DIR} src
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  )

use_skyai(oct)
include_directories(${SKYAI_INCLUDE_DIRS})
link_directories (${SKYAI_LIBRARY_DIRS})
add_skyai_executable(variable_binexec)
add_skyai_executable(variable_parser)
add_skyai_executable(variable_writer)
add_skyai_executable(bioloid01)

use_skyai(oct cv)
include_directories(${SKYAI_INCLUDE_DIRS})
link_directories (${SKYAI_LIBRARY_DIRS})
add_skyai_executable(marker-tracker)

use_skyai(skyai)
include_directories(${SKYAI_INCLUDE_DIRS})
link_directories (${SKYAI_LIBRARY_DIRS})
add_skyai_executable(agent_binexec)
add_skyai_executable(agent_parser)
add_skyai_executable(agent_writer)
add_skyai_executable(avf_plotter)


find_package(Boost COMPONENTS
  thread
  REQUIRED)
if(NOT Boost_THREAD_FOUND)
  message(WARNING, " Boost-thread library is not found." )
endif()

use_skyai(skyai cv)
include_directories(${SKYAI_INCLUDE_DIRS})
link_directories (${SKYAI_LIBRARY_DIRS})
add_skyai_executable(mtracker-module)


use_skyai(oct)
set(SKYAI_LIBRARY_DIRS ${SKYAI_LIBRARY_DIRS} ${SKYAI_LIBRARY_DIR})
include_directories(${SKYAI_INCLUDE_DIRS})
link_directories (${SKYAI_LIBRARY_DIRS})
add_skyai_executable(load-lib skyai)
# NOTE: libskyai is linked, but libskyai_mcore and libskyai_mstd are not linked

