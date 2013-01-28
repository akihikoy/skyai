use_skyai(oct)
include_directories(${SKYAI_INCLUDE_DIRS})
link_directories (${SKYAI_LIBRARY_DIRS})
add_executable (gen-bubble.out gen-bubble.cpp)
target_link_libraries (gen-bubble.out ${SKYAI_LIBRARIES})
set_target_properties(gen-bubble.out PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin/skyai/ngnet_generator)
install (TARGETS gen-bubble.out DESTINATION bin/skyai/ngnet_generator)

use_skyai(oct)
include_directories(${SKYAI_INCLUDE_DIRS})
link_directories (${SKYAI_LIBRARY_DIRS})
add_executable (gen-grid.out gen-grid.cpp)
target_link_libraries (gen-grid.out ${SKYAI_LIBRARIES})
set_target_properties(gen-grid.out PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin/skyai/ngnet_generator)
install (TARGETS gen-grid.out DESTINATION bin/skyai/ngnet_generator)
