find_package(Boost COMPONENTS
  thread
  REQUIRED)
if(NOT Boost_THREAD_FOUND)
  message(WARNING, " Boost-thread library is not found." )
endif()

use_skyai(skyai cv)
include_directories(${SKYAI_INCLUDE_DIRS})
link_directories (${SKYAI_LIBRARY_DIRS})

add_library (bioloid SHARED libbioloid.cpp)

add_executable (bioloid.out bioloid.cpp)
target_link_libraries (bioloid.out bioloid ${SKYAI_LIBRARIES})

execute_process (COMMAND
    ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/m
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  )

install (TARGETS bioloid LIBRARY DESTINATION lib/skyai)
install (TARGETS bioloid.out DESTINATION bin/skyai/bioloid)
install (DIRECTORY m/ DESTINATION bin/skyai/bioloid/m
    FILES_MATCHING PATTERN "*.agent" PATTERN "*.dat")
