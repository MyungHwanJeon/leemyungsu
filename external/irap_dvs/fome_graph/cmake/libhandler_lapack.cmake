macro (libhandler_lapack)
  # find_package(LAPACK) doesn't seem to be working on ubuntu 10.04, so revert to using find_library
  libhandler_find_library (lapack "on ubuntu `sudo apt-get install liblapack-dev`" ${ARGN})
  if (LAPACK_FOUND)
    set (IRPLIB_LAPACK ${LAPACK_LIBRARIES})
  endif ()
endmacro ()
