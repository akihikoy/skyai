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
install (DIRECTORY ./ DESTINATION include/skyai/3rdparty/markerdetection
    FILES_MATCHING PATTERN "*.h")
##------------------------------------------------------------------
