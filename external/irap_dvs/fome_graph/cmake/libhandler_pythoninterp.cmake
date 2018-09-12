macro (libhandler_pythoninterp)
  libhandler_find_package (PythonInterp "on ubuntu `sudo apt-get install python`" ${ARGN})
endmacro ()
