set (WITH_LIBOCTAVE ON CACHE BOOL "Include LibOctave support")
set (WITH_BOOST ON CACHE BOOL "Include Boost support")

set (WITH_ODE ON CACHE BOOL "Include ODE (Open Dynamics Engine) support (only for building benchmarks)")
set (WITH_OPENCV ON CACHE BOOL "Include OpenCV support (only for building benchmarks)")

if(WITH_LIBOCTAVE)
  include(FindLibOctave)
  if(NOT LIBOCTAVE_FOUND)
    message(WARNING, " Octave library is not found." )
  endif()
endif()

if(WITH_BOOST)
  find_package(Boost COMPONENTS
    filesystem
    regex
    REQUIRED)
  if(NOT Boost_FOUND)
    message(WARNING, " Boost library is not found." )
  endif()
  if(NOT Boost_FILESYSTEM_FOUND)
    message(WARNING, " Boost-filesystem library is not found." )
  endif()
  if(NOT Boost_REGEX_FOUND)
    message(WARNING, " Boost-regex library is not found." )
  endif()
endif()

if(WITH_ODE)
  include(FindODE)
  if(NOT ODE_FOUND)
    message(WARNING, " ODE library is not found." )
  endif()
  add_definitions(-DODE_MINOR_VERSION=10 -DdDOUBLE)
  find_package(OpenGL COMPONENTS
    filesystem
    REQUIRED)
  if(NOT (OPENGL_FOUND AND OPENGL_GLU_FOUND))
    message(WARNING, " ODE library requires OpenGL and GLU, but not found." )
  endif()
endif()

if(WITH_OPENCV)
  include(FindOpenCV)
  if(NOT OPENCV_FOUND)
    message(WARNING, " OpenCV library is not found." )
  endif()
endif()


if(PROJECT_NAME STREQUAL SKYAI)
  set (MARKERDETECTION_INCLUDE_DIRS ${SKYAI_SOURCE_DIR}/3rdparty/markerdetection/..)
  set (MARKERDETECTION_LIBRARY_DIRS ${SKYAI_BINARY_DIR}/3rdparty/markerdetection)
  set (MARKERDETECTION_LIBRARIES markerdetection)

  set (LORA_INCLUDE_DIR  ${SKYAI_SOURCE_DIR}/liblora/include)
  set (LORA_LIBRARY_DIR  ${SKYAI_BINARY_DIR}/liblora)

  set (SKYAI_INCLUDE_DIR  ${SKYAI_SOURCE_DIR}/libskyai/include)
  set (SKYAI_LIBRARY_DIR  ${SKYAI_BINARY_DIR}/libskyai)
else()
  set (SKYAI_INSTALL_DIR "" CACHE PATH "Path to the SkyAI's build directory or install directory")

  find_path(MARKERDETECTION_edgeldetector_INCLUDE_DIR  NAMES edgeldetector.h PATH_SUFFIXES markerdetection  PATHS ${SKYAI_INSTALL_DIR}/include/skyai/3rdparty /usr/include/skyai/3rdparty /usr/local/include/skyai/3rdparty NO_DEFAULT_PATH)
  find_path(MARKERDETECTION_LIBRARY_DIR  NAMES libmarkerdetection.so PATH_SUFFIXES PATHS ${SKYAI_INSTALL_DIR}/lib/skyai /usr/lib/skyai /usr/local/lib/skyai NO_DEFAULT_PATH)
  set (MARKERDETECTION_LIBRARIES markerdetection)
  set (MARKERDETECTION_INCLUDE_DIR ${MARKERDETECTION_edgeldetector_INCLUDE_DIR}/..)
  mark_as_advanced(MARKERDETECTION_edgeldetector_INCLUDE_DIR)
  mark_as_advanced(MARKERDETECTION_LIBRARY_DIR)
  if(NOT MARKERDETECTION_edgeldetector_INCLUDE_DIR)
    set(MARKERDETECTION_edgeldetector_INCLUDE_DIR "MARKERDETECTION_edgeldetector_INCLUDE_DIR-NOTFOUND" CACHE PATH "Path to edgeldetector.h" FORCE)
    message (WARNING "edgeldetector.h not found.")
  endif()
  if(NOT MARKERDETECTION_LIBRARY_DIR)
    set(MARKERDETECTION_LIBRARY_DIR "MARKERDETECTION_LIBRARY_DIR-NOTFOUND" CACHE PATH "Path to libmarkerdetection.so" FORCE)
    message (WARNING "libmarkerdetection.so not found.")
  endif()

  find_path(LORA_common_INCLUDE_DIR  NAMES common.h PATH_SUFFIXES lora  PATHS ${SKYAI_INSTALL_DIR}/include/skyai /usr/include/skyai /usr/local/include/skyai NO_DEFAULT_PATH)
  find_path(LORA_LIBRARY_DIR  NAMES liblora_std.so PATH_SUFFIXES PATHS ${SKYAI_INSTALL_DIR}/lib/skyai /usr/lib/skyai /usr/local/lib/skyai NO_DEFAULT_PATH)
  set (LORA_INCLUDE_DIR ${LORA_common_INCLUDE_DIR}/..)
  mark_as_advanced(LORA_common_INCLUDE_DIR)
  mark_as_advanced(LORA_LIBRARY_DIR)
  if(NOT LORA_common_INCLUDE_DIR)
    set(LORA_common_INCLUDE_DIR "LORA_common_INCLUDE_DIR-NOTFOUND" CACHE PATH "Path to loca/common.h" FORCE)
    message (WARNING "loca/common.h not found.")
  endif()
  if(NOT LORA_LIBRARY_DIR)
    set(LORA_LIBRARY_DIR "LORA_LIBRARY_DIR-NOTFOUND" CACHE PATH "Path to liblora_std.so" FORCE)
    message (WARNING "liblora_std.so not found.")
  endif()

  find_path(SKYAI_base_INCLUDE_DIR NAMES base.h PATH_SUFFIXES skyai PATHS ${SKYAI_INSTALL_DIR}/include/skyai /usr/include/skyai /usr/local/include/skyai NO_DEFAULT_PATH)
  find_path(SKYAI_LIBRARY_DIR NAMES libskyai.so PATH_SUFFIXES PATHS ${SKYAI_INSTALL_DIR}/lib/skyai /usr/lib/skyai /usr/local/lib/skyai NO_DEFAULT_PATH)
  set (SKYAI_INCLUDE_DIR ${SKYAI_base_INCLUDE_DIR}/..)
  mark_as_advanced(SKYAI_base_INCLUDE_DIR)
  mark_as_advanced(SKYAI_LIBRARY_DIR)
  if(NOT SKYAI_base_INCLUDE_DIR)
    set(SKYAI_base_INCLUDE_DIR "SKYAI_base_INCLUDE_DIR-NOTFOUND" CACHE PATH "Path to skyai/base.h" FORCE)
    message (WARNING "skyai/base.h not found.")
  endif()
  if(NOT SKYAI_LIBRARY_DIR)
    set(SKYAI_LIBRARY_DIR "SKYAI_LIBRARY_DIR-NOTFOUND" CACHE PATH "Path to SKYAI_LIBRARY_DIR" FORCE)
    message (WARNING "SKYAI_LIBRARY_DIR not found.")
  endif()

  if (LORA_common_INCLUDE_DIR AND LORA_LIBRARY_DIR
      AND SKYAI_base_INCLUDE_DIR AND SKYAI_LIBRARY_DIR)
    message (STATUS "SkyAI found")
  else()
    message (WARNING "Part of SkyAI not found. Specify: SKYAI_INSTALL_DIR")
  endif()
endif()


# usage: use_skyai([skyai][oct][ode][cv])
# note: using liblora is default
# this function sets SKYAI_INCLUDE_DIRS, SKYAI_LIBRARY_DIRS, SKYAI_LIBRARIES
function(use_skyai)
  set (USING_SKYAI "NO")
  set (USING_OCT   "NO")
  set (USING_ODE   "NO")
  set (USING_CV    "NO")
  foreach(a ${ARGN})
    if(${a} STREQUAL skyai)
      set (USING_SKYAI "YES")
      set (USING_OCT   "YES")
    elseif(${a} STREQUAL oct)
      set (USING_OCT   "YES")
    elseif(${a} STREQUAL ode)
      set (USING_ODE   "YES")
    elseif(${a} STREQUAL cv)
      set (USING_CV    "YES")
    endif()
  endforeach()

  set(L_INCLUDE_DIRS
      ${L_INCLUDE_DIRS}
      ${LORA_INCLUDE_DIR}
      ${Boost_INCLUDE_DIRS}
    )
  set(L_LIBRARY_DIRS
      ${L_LIBRARY_DIRS}
      ${LORA_LIBRARY_DIR}
      ${Boost_LIBRARY_DIRS}
    )
  set(L_LIBRARIES
      ${L_LIBRARIES}
      lora_std
      ${Boost_LIBRARIES}
      ${CMAKE_DL_LIBS}
    )

  if(USING_OCT)
    set(L_INCLUDE_DIRS
        ${L_INCLUDE_DIRS}
        ${LIBOCTAVE_INCLUDE_DIRS}
      )
    set(L_LIBRARY_DIRS
        ${L_LIBRARY_DIRS}
        ${LIBOCTAVE_LIBRARY_DIRS}
      )
    set(L_LIBRARIES
        ${L_LIBRARIES}
        lora_oct
        ${LIBOCTAVE_LIBRARIES}
      )
  endif()

  if(USING_ODE)
    set(L_INCLUDE_DIRS
        ${L_INCLUDE_DIRS}
        ${ODE_INCLUDE_DIRS}
        ${OPENGL_INCLUDE_DIRS}
      )
    set(L_LIBRARY_DIRS
        ${L_LIBRARY_DIRS}
        ${ODE_LIBRARY_DIRS}
        ${OPENGL_LIBRARY_DIRS}
      )
    set(L_LIBRARIES
        ${L_LIBRARIES}
        lora_ode
        ${ODE_LIBRARIES}
        ${OPENGL_LIBRARIES}
      )
  endif()

  if(USING_CV)
    set(L_INCLUDE_DIRS
        ${L_INCLUDE_DIRS}
        ${OPENCV_INCLUDE_DIRS}
        ${MARKERDETECTION_INCLUDE_DIRS}
      )
    set(L_LIBRARY_DIRS
        ${L_LIBRARY_DIRS}
        ${OPENCV_LIBRARY_DIRS}
        ${MARKERDETECTION_LIBRARY_DIRS}
      )
    set(L_LIBRARIES
        ${L_LIBRARIES}
        lora_cv
        ${OPENCV_LIBRARIES}
        ${MARKERDETECTION_LIBRARIES}
      )
  endif()

  if(USING_SKYAI)
    set(L_INCLUDE_DIRS
        ${L_INCLUDE_DIRS}
        ${SKYAI_INCLUDE_DIR}
      )
    set(L_LIBRARY_DIRS
        ${L_LIBRARY_DIRS}
        ${SKYAI_LIBRARY_DIR}
      )
    set(L_LIBRARIES
        ${L_LIBRARIES}
        skyai
      )
  endif()

  set(SKYAI_INCLUDE_DIRS  ${L_INCLUDE_DIRS} PARENT_SCOPE)
  set(SKYAI_LIBRARY_DIRS  ${L_LIBRARY_DIRS} PARENT_SCOPE)
  set(SKYAI_LIBRARIES     ${L_LIBRARIES}    PARENT_SCOPE)
endfunction()
