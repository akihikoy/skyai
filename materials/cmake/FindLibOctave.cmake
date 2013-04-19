set (LIBOCTAVE_INSTALL_DIR "" CACHE PATH "Path to the directory where you installed LibOctave")

find_path(LIBOCTAVE_octave_INCLUDE_DIR NAMES octave.h PATH_SUFFIXES octave PATHS ${LIBOCTAVE_INSTALL_DIR}/include /usr/include /usr/local/include  NO_DEFAULT_PATH)
if(NOT LIBOCTAVE_octave_INCLUDE_DIR)
  set(LIBOCTAVE_octave_INCLUDE_DIR "LIBOCTAVE_octave_INCLUDE_DIR-NOTFOUND" CACHE PATH "Path to the directory that contains octave.h" FORCE)
  message (WARNING "octave.h not found.")
endif()

file(GLOB LIBOCTAVE_octave_PATHS "${LIBOCTAVE_INSTALL_DIR}"/lib/octave*;/usr/lib/octave*;/usr/local/lib/octave*)
# message(STATUS "LIBOCTAVE_octave_PATHS: ${LIBOCTAVE_octave_PATHS}")

find_library(LIBOCTAVE_octave_LIBRARY NAMES octave PATHS ${LIBOCTAVE_octave_PATHS} ${LIBOCTAVE_INSTALL_DIR}/lib /usr/lib /usr/local/lib NO_DEFAULT_PATH)
if(NOT LIBOCTAVE_octave_LIBRARY)
  set(LIBOCTAVE_octave_LIBRARY "LIBOCTAVE_octave_LIBRARY-NOTFOUND" CACHE PATH "Path to liboctave" FORCE)
  message (WARNING "liboctave not found.")
endif()

set(LIBOCTAVE_dl_LIBRARY ${CMAKE_DL_LIBS})

find_library(LIBOCTAVE_fttw3_LIBRARY  NAMES fttw3  PATHS ${LIBOCTAVE_INSTALL_DIR}/lib /usr/lib /usr/local/lib NO_DEFAULT_PATH)
if(NOT LIBOCTAVE_fttw3_LIBRARY)
  set(LIBOCTAVE_fttw3_LIBRARY "")
endif()
find_library(LIBOCTAVE_atlas_LIBRARY  NAMES atlas  PATHS ${LIBOCTAVE_INSTALL_DIR}/lib /usr/lib /usr/local/lib NO_DEFAULT_PATH)
find_library(LIBOCTAVE_lapack_LIBRARY NAMES lapack PATHS ${LIBOCTAVE_INSTALL_DIR}/lib /usr/lib /usr/local/lib NO_DEFAULT_PATH)
find_library(LIBOCTAVE_blas_LIBRARY   NAMES blas   PATHS ${LIBOCTAVE_INSTALL_DIR}/lib /usr/lib /usr/local/lib NO_DEFAULT_PATH)

set (LIBOCTAVE_FOUND "NO")
if(LIBOCTAVE_octave_INCLUDE_DIR AND LIBOCTAVE_octave_LIBRARY)
  set (LIBOCTAVE_FOUND "YES")
  message (STATUS "LibOctave found")
else()
  message (WARNING "LibOctave not found. Specify: LIBOCTAVE_INSTALL_DIR")
endif()

mark_as_advanced(LIBOCTAVE_octave_INCLUDE_DIR)
mark_as_advanced(LIBOCTAVE_octave_LIBRARY)
mark_as_advanced(LIBOCTAVE_dl_LIBRARY)
mark_as_advanced(LIBOCTAVE_fttw3_LIBRARY)
mark_as_advanced(LIBOCTAVE_atlas_LIBRARY)
mark_as_advanced(LIBOCTAVE_lapack_LIBRARY)
mark_as_advanced(LIBOCTAVE_blas_LIBRARY)

set (LIBOCTAVE_INCLUDE_DIRS ${LIBOCTAVE_octave_INCLUDE_DIR}/..)
set (LIBOCTAVE_LIBRARIES
    ${LIBOCTAVE_octave_LIBRARY}
    ${LIBOCTAVE_dl_LIBRARY}
    ${LIBOCTAVE_fttw3_LIBRARY}
    ${LIBOCTAVE_atlas_LIBRARY}
    ${LIBOCTAVE_lapack_LIBRARY}
    ${LIBOCTAVE_blas_LIBRARY}
  )

