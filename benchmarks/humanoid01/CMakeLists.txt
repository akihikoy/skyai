use_skyai(skyai ode)
include_directories(${SKYAI_INCLUDE_DIRS})
link_directories (${SKYAI_LIBRARY_DIRS})

add_library (humanoid01 SHARED libhumanoid01.cpp)

add_executable (humanoid01.out humanoid01.cpp)
target_link_libraries (humanoid01.out humanoid01 ${SKYAI_LIBRARIES})

execute_process (COMMAND
    ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/m
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  )

install (TARGETS humanoid01 LIBRARY DESTINATION lib/skyai)
install (TARGETS humanoid01.out DESTINATION bin/skyai/humanoid01)
install (DIRECTORY m/ DESTINATION bin/skyai/humanoid01/m
    FILES_MATCHING PATTERN "*.agent" PATTERN "*.dat" PATTERN "*.ppm")
