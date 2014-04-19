##------------------------------------------------------------------
set(lora_std_srcs
    src/common.cpp
    src/binary.cpp
    src/bioloid.cpp
    src/boost_filesystem_ext.cpp
    src/cast.cpp
    src/file.cpp
    src/math.cpp
    src/rand.cpp
    src/serial.cpp
    src/setting_file.cpp
    src/small_classes.cpp
    src/stl_ext.cpp
    src/stl_math.cpp
    src/stl_rand.cpp
    src/string.cpp
    src/string_list.cpp
    src/string_list_ext.cpp
    src/sys.cpp
    src/type_gen.cpp
    src/variable_any.cpp
    src/variable_bindef.cpp
    src/variable_binexec.cpp
    src/variable_literal.cpp
    src/variable_parser.cpp
    src/variable_space.cpp
  )
use_skyai()
include_directories (${SKYAI_INCLUDE_DIRS})
add_library (lora_std SHARED ${lora_std_srcs})
set_target_properties (lora_std PROPERTIES SOVERSION ${SKYAI_SOVERSION})
target_link_libraries (lora_std ${SKYAI_LIBRARIES})
install (TARGETS lora_std LIBRARY DESTINATION lib/skyai)
##------------------------------------------------------------------
set(lora_oct_srcs
    src/octave.cpp
    src/octave_str.cpp
    src/ctrl_tools.cpp
    src/oldngnet.cpp
    src/type_gen_oct.cpp
    src/variable_space_oct.cpp
  )
if(WITH_LIBOCTAVE)
  use_skyai(oct)
  include_directories (${SKYAI_INCLUDE_DIRS})
  add_library (lora_oct SHARED ${lora_oct_srcs})
  set_target_properties (lora_oct PROPERTIES SOVERSION ${SKYAI_SOVERSION})
  target_link_libraries (lora_oct ${SKYAI_LIBRARIES})
  install (TARGETS lora_oct LIBRARY DESTINATION lib/skyai)
endif()
##------------------------------------------------------------------
set(lora_ode_srcs
    src/ode.cpp
    src/ode_ds.cpp
    src/robot_model.cpp
  )
if(WITH_ODE)
  use_skyai(ode)
  include_directories (${SKYAI_INCLUDE_DIRS})
  add_library (lora_ode SHARED ${lora_ode_srcs})
  set_target_properties (lora_ode PROPERTIES SOVERSION ${SKYAI_SOVERSION})
  target_link_libraries (lora_ode ${SKYAI_LIBRARIES})
  install (TARGETS lora_ode LIBRARY DESTINATION lib/skyai)
endif()
##------------------------------------------------------------------
set(lora_cv_srcs
    src/cv.cpp
    src/marker_tracker.cpp
  )
if(WITH_OPENCV)
  use_skyai(cv)
  include_directories (${SKYAI_INCLUDE_DIRS})
  add_library (lora_cv SHARED ${lora_cv_srcs})
  set_target_properties (lora_cv PROPERTIES SOVERSION ${SKYAI_SOVERSION})
  target_link_libraries (lora_cv ${SKYAI_LIBRARIES})
  install (TARGETS lora_cv LIBRARY DESTINATION lib/skyai)
endif()
##------------------------------------------------------------------
execute_process (
    COMMAND mkdir -p include/skyai
    COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/include/lora include/skyai/
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
  )
install (DIRECTORY include/ DESTINATION include/skyai
    FILES_MATCHING PATTERN "*.h")
##------------------------------------------------------------------
