macro (libhandler_m)
  libhandler_find_library (m "on ubuntu `sudo apt-get install build-essential`" ${ARGN})
  if (M_FOUND)
    set (IRPLIB_M ${M_LIBRARIES})
  endif ()
endmacro ()
