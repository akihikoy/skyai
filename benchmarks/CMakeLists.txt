set (BUILD_BENCHMARKS_maze2d ON CACHE BOOL "Build maze2d")
set (BUILD_BENCHMARKS_humanoid01 ON CACHE BOOL "Build humanoid01")
set (BUILD_BENCHMARKS_dynsim ON CACHE BOOL "Build dynsim")
set (BUILD_BENCHMARKS_bioloid ON CACHE BOOL "Build bioloid")

if (BUILD_BENCHMARKS_maze2d)
  use_skyai(skyai)
  include_directories(${SKYAI_INCLUDE_DIRS})
  link_directories (${SKYAI_LIBRARY_DIRS})

  add_library (maze2d SHARED maze2d/libmaze2d.cpp)
  add_library (maze2d_main SHARED maze2d/maze2d.cpp)
  set_target_properties (maze2d PROPERTIES SOVERSION ${SKYAI_SOVERSION})
  set_target_properties (maze2d_main PROPERTIES SOVERSION ${SKYAI_SOVERSION})
  target_link_libraries (maze2d ${SKYAI_LIBRARIES})
  target_link_libraries (maze2d_main maze2d)

  install (TARGETS maze2d LIBRARY DESTINATION lib/skyai)
  install (TARGETS maze2d_main LIBRARY DESTINATION lib/skyai)
endif()

if (BUILD_BENCHMARKS_humanoid01)
  use_skyai(skyai ode)
  include_directories(${SKYAI_INCLUDE_DIRS})
  link_directories (${SKYAI_LIBRARY_DIRS})

  add_library (humanoid01 SHARED humanoid01/libhumanoid01.cpp)
  add_library (humanoid01_main SHARED humanoid01/humanoid01.cpp)
  set_target_properties (humanoid01 PROPERTIES SOVERSION ${SKYAI_SOVERSION})
  set_target_properties (humanoid01_main PROPERTIES SOVERSION ${SKYAI_SOVERSION})
  target_link_libraries (humanoid01 ${SKYAI_LIBRARIES})
  target_link_libraries (humanoid01_main humanoid01)

  install (TARGETS humanoid01 LIBRARY DESTINATION lib/skyai)
  install (TARGETS humanoid01_main LIBRARY DESTINATION lib/skyai)
endif()

if (BUILD_BENCHMARKS_dynsim)
  use_skyai(skyai ode)
  include_directories(${SKYAI_INCLUDE_DIRS})
  link_directories (${SKYAI_LIBRARY_DIRS})

  add_library (dynsim SHARED dynsim/libdynsim.cpp)
  add_library (dynsim_main SHARED dynsim/dynsim.cpp)
  set_target_properties (dynsim PROPERTIES SOVERSION ${SKYAI_SOVERSION})
  set_target_properties (dynsim_main PROPERTIES SOVERSION ${SKYAI_SOVERSION})
  target_link_libraries (dynsim ${SKYAI_LIBRARIES})
  target_link_libraries (dynsim_main dynsim)

  install (TARGETS dynsim LIBRARY DESTINATION lib/skyai)
  install (TARGETS dynsim_main LIBRARY DESTINATION lib/skyai)
endif()

if (BUILD_BENCHMARKS_bioloid)
  find_package(Boost COMPONENTS
    thread
    REQUIRED)
  if(NOT Boost_THREAD_FOUND)
    message(WARNING, " Boost-thread library is not found." )
  endif()

  use_skyai(skyai cv)
  include_directories(${SKYAI_INCLUDE_DIRS})
  link_directories (${SKYAI_LIBRARY_DIRS})

  add_library (bioloid SHARED bioloid/libbioloid.cpp)
  add_library (bioloid_main SHARED bioloid/bioloid.cpp)
  set_target_properties (bioloid PROPERTIES SOVERSION ${SKYAI_SOVERSION})
  set_target_properties (bioloid_main PROPERTIES SOVERSION ${SKYAI_SOVERSION})
  target_link_libraries (bioloid ${SKYAI_LIBRARIES})
  target_link_libraries (bioloid_main bioloid)

  install (TARGETS bioloid LIBRARY DESTINATION lib/skyai)
  install (TARGETS bioloid_main LIBRARY DESTINATION lib/skyai)
endif()

# for testing in the build directory
execute_process (
    COMMAND mkdir -p share/skyai
    COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/agents share/skyai/
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
  )

install (DIRECTORY agents/ DESTINATION share/skyai/agents
    FILES_MATCHING PATTERN "*.agent" PATTERN "*.dat" PATTERN "*.ppm" PATTERN "*.png")
