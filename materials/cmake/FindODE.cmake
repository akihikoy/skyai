set (ODE_BUILD_DIR "" CACHE PATH "Path to the directory where you built ODE")
set (ODE_INSTALL_DIR "" CACHE PATH "Path to the directory where you installed ODE")

find_path(ODE_ode_INCLUDE_DIR NAMES ode.h PATH_SUFFIXES ode PATHS ${ODE_INSTALL_DIR}/include ${ODE_BUILD_DIR}/include /usr/include /usr/local/include NO_DEFAULT_PATH)
if(NOT ODE_ode_INCLUDE_DIR)
  set(ODE_ode_INCLUDE_DIR "ODE_ode_INCLUDE_DIR-NOTFOUND" CACHE PATH "Path to the directory that contains ode.h" FORCE)
  message (WARNING "ode.h not found.")
endif()

find_path(ODE_drawstuff_INCLUDE_DIR NAMES drawstuff.h PATH_SUFFIXES drawstuff PATHS ${ODE_INSTALL_DIR}/include ${ODE_BUILD_DIR}/include /usr/include /usr/local/include NO_DEFAULT_PATH)
if(NOT ODE_drawstuff_INCLUDE_DIR)
  set(ODE_drawstuff_INCLUDE_DIR "ODE_drawstuff_INCLUDE_DIR-NOTFOUND" CACHE PATH "Path to the directory that contains drawstuff.h" FORCE)
  message (WARNING "drawstuff.h not found.")
endif()

find_library(ODE_ode_LIBRARY NAMES ode PATHS ${ODE_INSTALL_DIR}/lib ${ODE_BUILD_DIR}/ode/src/.libs /usr/lib /usr/local/lib NO_DEFAULT_PATH)
if(NOT ODE_ode_LIBRARY)
  set(ODE_ode_LIBRARY "ODE_ode_LIBRARY-NOTFOUND" CACHE PATH "Path to libode" FORCE)
  message (WARNING "libode not found.")
endif()
find_library(ODE_drawstuff_LIBRARY NAMES drawstuff PATHS ${ODE_INSTALL_DIR}/lib ${ODE_BUILD_DIR}/drawstuff/src/.libs /usr/lib /usr/local/lib NO_DEFAULT_PATH)
if(NOT ODE_drawstuff_LIBRARY)
  set(ODE_drawstuff_LIBRARY "ODE_drawstuff_LIBRARY-NOTFOUND" CACHE PATH "Path to libdrawstuff" FORCE)
  message (WARNING "libdrawstuff not found.")
endif()

set (ODE_FOUND "NO")
if(ODE_ode_INCLUDE_DIR AND ODE_drawstuff_INCLUDE_DIR
    AND ODE_ode_LIBRARY AND ODE_drawstuff_LIBRARY)
  set (ODE_FOUND "YES")
  message (STATUS "ODE found")
else()
  message (WARNING "ODE not found. Specify: ODE_BUILD_DIR or ODE_INSTALL_DIR")
endif()

mark_as_advanced(ODE_ode_INCLUDE_DIR)
mark_as_advanced(ODE_drawstuff_INCLUDE_DIR)
mark_as_advanced(ODE_ode_LIBRARY)
mark_as_advanced(ODE_drawstuff_LIBRARY)

set (ODE_INCLUDE_DIRS
    ${ODE_ode_INCLUDE_DIR}/..
    ${ODE_drawstuff_INCLUDE_DIR}/..
  )
set (ODE_LIBRARIES
    ${ODE_ode_LIBRARY}
    ${ODE_drawstuff_LIBRARY}
  )
