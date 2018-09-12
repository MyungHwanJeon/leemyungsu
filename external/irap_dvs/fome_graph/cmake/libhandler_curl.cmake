macro (libhandler_curl)
  libhandler_find_package (CURL "
on ubuntu <12.04 `sudo apt-get install libcurl4-dev`
on ubuntu >=12.04 `sudo apt-get install libcurl4-nss-dev`"
${ARGN})
  if (CURL_FOUND)
    include_directories (${CURL_INCLUDE_DIR})
    set (IRPLIB_CURL ${CURL_LIBRARIES})
  endif ()
endmacro ()
