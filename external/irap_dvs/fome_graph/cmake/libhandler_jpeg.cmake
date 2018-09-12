macro (libhandler_jpeg)
  libhandler_find_package (JPEG "on ubuntu `sudo apt-get install libjpeg-dev`" ${ARGN})
  if (JPEG_FOUND)
    include_directories (${JPEG_INCLUDE_DIR})
    set (IRPLIB_JPEG ${JPEG_LIBRARIES})
  endif ()
endmacro ()
