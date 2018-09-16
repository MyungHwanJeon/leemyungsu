macro (libhandler_gps)
  libhandler_find_library (gps "on ubuntu `sudo apt-get install libgps-dev`" ${ARGN})
  if (GPS_FOUND)
    set (IRPLIB_GPS ${GPS_LIBRARIES})
  endif ()
endmacro ()