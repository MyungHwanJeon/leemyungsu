macro (libhandler_ann)
  libhandler_find_library (ann "on ubuntu `sudo apt-get install libann-dev`" ${ARGN})
  if (ANN_FOUND)
    set (IRPLIB_ANN ${ANN_LIBRARIES})
  endif ()
endmacro ()
