set (BUILD_BENCHMARKS_maze2d ON CACHE BOOL "Build maze2d")
set (BUILD_BENCHMARKS_humanoid01 ON CACHE BOOL "Build humanoid01")
set (BUILD_BENCHMARKS_bioloid ON CACHE BOOL "Build bioloid")

if (BUILD_BENCHMARKS_maze2d)
  add_subdirectory (maze2d)
endif()
if (BUILD_BENCHMARKS_humanoid01)
  add_subdirectory (humanoid01)
endif()
if (BUILD_BENCHMARKS_bioloid)
  add_subdirectory (bioloid)
endif()

execute_process (COMMAND
    ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/cmn
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  )

install (DIRECTORY cmn/ DESTINATION bin/skyai/cmn
    FILES_MATCHING PATTERN "*.agent" PATTERN "*.dat")
