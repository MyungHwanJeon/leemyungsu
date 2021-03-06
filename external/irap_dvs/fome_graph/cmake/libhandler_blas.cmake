macro (libhandler_blas)
  # find_package(BLAS) doesn't seem to be working on ubuntu 10.04, so revert to using find_library()
  libhandler_find_library (blas "on ubuntu `sudo apt-get install libblas-dev`" ${ARGN})
  if (BLAS_FOUND)
    set (IRPLIB_BLAS ${BLAS_LIBRARIES})
  endif ()
endmacro ()
