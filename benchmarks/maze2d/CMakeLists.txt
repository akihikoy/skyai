use_skyai(skyai)
include_directories(${SKYAI_INCLUDE_DIRS})
link_directories (${SKYAI_LIBRARY_DIRS})

add_library (maze2d SHARED libmaze2d.cpp)

add_library (maze2d_main SHARED maze2d.cpp)
target_link_libraries (maze2d_main maze2d ${SKYAI_LIBRARIES})

execute_process (COMMAND
    ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/m
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  )

install (TARGETS maze2d LIBRARY DESTINATION lib/skyai)
install (TARGETS maze2d_main LIBRARY DESTINATION lib/skyai)
install (DIRECTORY m/ DESTINATION bin/skyai/maze2d/m
    FILES_MATCHING PATTERN "*.agent" PATTERN "*.dat")
