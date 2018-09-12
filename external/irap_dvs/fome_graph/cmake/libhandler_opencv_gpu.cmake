macro (libhandler_opencv_gpu)
  libhandler_find_library (opencv_gpu "\nYou must install opencv 2.3 with CUDA enabled.\n" ${ARGN})
  if (OPENCV_GPU_FOUND)
    add_definitions (-DOPENCV_GPU_FOUND)
    set (IRPLIB_OPENCV_GPU ${OPENCV_GPU_LIBRARIES})
  endif ()
endmacro ()
