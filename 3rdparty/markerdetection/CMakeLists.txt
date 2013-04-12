##------------------------------------------------------------------
set(markerdetection_srcs
    buffer.cpp
    edgel.cpp
    edgeldetector.cpp
    linesegment.cpp
  )
include_directories (
    include
  )
add_library (markerdetection SHARED ${markerdetection_srcs})
install (TARGETS markerdetection LIBRARY DESTINATION lib/skyai)
##------------------------------------------------------------------
execute_process (
    COMMAND mkdir -p include/skyai/3rdparty
    COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR} include/skyai/3rdparty/
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
  )
install (DIRECTORY ./ DESTINATION include/skyai/3rdparty/markerdetection
    FILES_MATCHING PATTERN "*.h")
##------------------------------------------------------------------
