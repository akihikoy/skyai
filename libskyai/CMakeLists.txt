##------------------------------------------------------------------
file(GLOB skyai_srcs "src/*.cpp")
use_skyai(skyai)
include_directories(${SKYAI_INCLUDE_DIRS})
add_library (skyai SHARED ${skyai_srcs})
install (TARGETS skyai LIBRARY DESTINATION lib/skyai)
##------------------------------------------------------------------
file(GLOB skyai_mcore_srcs "src/modules_core/*.cpp")
use_skyai(skyai)
include_directories(${SKYAI_INCLUDE_DIRS})
add_library (skyai_mcore SHARED ${skyai_mcore_srcs})
install (TARGETS skyai_mcore LIBRARY DESTINATION lib/skyai)
##------------------------------------------------------------------
file(GLOB skyai_mstd_srcs "src/modules_std/*.cpp")
use_skyai(skyai)
include_directories(${SKYAI_INCLUDE_DIRS})
add_library (skyai_mstd SHARED ${skyai_mstd_srcs})
install (TARGETS skyai_mstd LIBRARY DESTINATION lib/skyai)
##------------------------------------------------------------------
use_skyai(skyai)
include_directories(${SKYAI_INCLUDE_DIRS})
add_executable (general_agent.out src/execs/general_agent.cpp)
target_link_libraries (general_agent.out ${SKYAI_LIBRARIES})
install (TARGETS general_agent.out DESTINATION bin/skyai)
##------------------------------------------------------------------
execute_process (
    COMMAND mkdir -p include/skyai
    COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/include/skyai include/skyai/
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
  )
install (DIRECTORY include/ DESTINATION include/skyai
    FILES_MATCHING PATTERN "*.h")
##------------------------------------------------------------------
