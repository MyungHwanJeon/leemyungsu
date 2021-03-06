# ------------------------------------------------------------------------------------
# Helper to use PCL from outside project
#
# target_link_libraries(my_fabulous_target PCL_XXX_LIBRARIES) where XXX is the 
# upper cased xxx from : 
# 
# - common
# - kdtree
# - sample_consensus
# - octree
# - search
# - range_image
# - features
# - registration
# - filters
# - io
# - segmentation
# - surface
# - visualization
# - keypoints
# - people
# - tracking
# - apps
#
# PCL_INCLUDE_DIRS is filled with PCL and available 3rdparty headers
# PCL_LIBRARY_DIRS is filled with PCL components libraries install directory and
# 3rdparty libraries paths
# 
#                                   www.pointclouds.org
#------------------------------------------------------------------------------------

### ---[ some useful macros

macro(pcl_report_not_found _reason)
  unset(PCL_FOUND)
  unset(PCL_LIBRARIES)
  unset(PCL_INCLUDE_DIRS)
  unset(PCL_LIBRARY_DIRS)
  unset(PCL_DEFINITONS)
  if(PCL_FIND_REQUIRED)
    message(FATAL_ERROR ${_reason})
  elseif(NOT PCL_FIND_QUIETLY)
    #message(WARNING ${_reason})
  endif()
  return()
endmacro(pcl_report_not_found)

macro(pcl_message)
  if(NOT PCL_FIND_QUIETLY)
    message(${ARGN})
  endif(NOT PCL_FIND_QUIETLY)
endmacro(pcl_message)

macro(find_boost)
  if(PCL_ALL_IN_ONE_INSTALLER)
    set(BOOST_ROOT "${PCL_ROOT}/3rdParty/Boost")
  elseif(NOT BOOST_INCLUDEDIR)
    set(BOOST_INCLUDEDIR "/usr/include")
  endif(PCL_ALL_IN_ONE_INSTALLER)
  find_package(Boost 1.40.0 ${QUIET_} COMPONENTS system filesystem thread date_time iostreams)
  set(BOOST_FOUND ${Boost_FOUND})
  set(BOOST_INCLUDE_DIRS "${Boost_INCLUDE_DIR}")
  set(BOOST_LIBRARY_DIRS "${Boost_LIBRARY_DIRS}")
  set(BOOST_LIBRARIES ${Boost_LIBRARIES})
endmacro(find_boost)

#remove this as soon as eigen is shipped with FindEigen.cmake
macro(find_eigen)
  if(PCL_ALL_IN_ONE_INSTALLER)
    set(EIGEN_ROOT "${PCL_ROOT}/3rdParty/Eigen")
  elseif(NOT EIGEN_ROOT)
    get_filename_component(EIGEN_ROOT "/usr/include/eigen3" PATH)
  endif(PCL_ALL_IN_ONE_INSTALLER)
  if(PkgConfig_FOUND)
    pkg_check_modules(PC_EIGEN eigen3)
  endif(PkgConfig_FOUND)
  find_path(EIGEN_INCLUDE_DIRS Eigen/Core
    HINTS ${PC_EIGEN_INCLUDEDIR} ${PC_EIGEN_INCLUDE_DIRS} 
          "${EIGEN_ROOT}" "$ENV{EIGEN_ROOT}"
    PATHS "$ENV{PROGRAMFILES}/Eigen 3.0.0" "$ENV{PROGRAMW6432}/Eigen 3.0.0"
          "$ENV{PROGRAMFILES}/Eigen" "$ENV{PROGRAMW6432}/Eigen"	  
    PATH_SUFFIXES eigen3 include/eigen3 include)
  find_package_handle_standard_args(eigen DEFAULT_MSG EIGEN_INCLUDE_DIRS)
endmacro(find_eigen)

#remove this as soon as qhull is shipped with FindQhull.cmake
macro(find_qhull)
  if(PCL_ALL_IN_ONE_INSTALLER)
    set(QHULL_ROOT "${PCL_ROOT}/3rdParty/Qhull")
  elseif(NOT QHULL_ROOT)
    get_filename_component(QHULL_ROOT "/usr/include/qhull" PATH)
  endif(PCL_ALL_IN_ONE_INSTALLER)
  set(QHULL_MAJOR_VERSION 6)

  find_path(QHULL_INCLUDE_DIRS
    NAMES libqhull/libqhull.h qhull.h
    HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
    PATHS "$ENV{PROGRAMFILES}/qhull 6.2.0.1373" "$ENV{PROGRAMW6432}/qhull 6.2.0.1373" 
          "$ENV{PROGRAMFILES}/qhull" "$ENV{PROGRAMW6432}/qhull" 
    PATH_SUFFIXES qhull src/libqhull libqhull include)

  # Most likely we are on windows so prefer static libraries over shared ones (Mourad's recommend)
  find_library(QHULL_LIBRARY 
    NAMES qhullstatic qhull qhull${QHULL_MAJOR_VERSION}
    HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
    PATHS "$ENV{PROGRAMFILES}/qhull 6.2.0.1373" "$ENV{PROGRAMW6432}/qhull 6.2.0.1373" 
          "$ENV{PROGRAMFILES}/qhull" "$ENV{PROGRAMW6432}/qhull" 
    PATH_SUFFIXES project build bin lib)
  
  find_library(QHULL_LIBRARY_DEBUG 
    NAMES qhullstatic_d qhull_d qhull${QHULL_MAJOR_VERSION}_d 
          qhull qhull${QHULL_MAJOR_VERSION}
    HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
    PATHS "$ENV{PROGRAMFILES}/qhull 6.2.0.1373" "$ENV{PROGRAMW6432}/qhull 6.2.0.1373" 
          "$ENV{PROGRAMFILES}/qhull" "$ENV{PROGRAMW6432}/qhull" 
    PATH_SUFFIXES project build bin lib)
  
  if(NOT QHULL_LIBRARY_DEBUG)
    set(QHULL_LIBRARY_DEBUG ${QHULL_LIBRARY})
  endif(NOT QHULL_LIBRARY_DEBUG)

  set(QHULL_LIBRARIES optimized ${QHULL_LIBRARY} debug ${QHULL_LIBRARY_DEBUG})

  find_package_handle_standard_args(qhull DEFAULT_MSG QHULL_LIBRARY QHULL_INCLUDE_DIRS)

  if(QHULL_FOUND)
    get_filename_component(QHULL_LIBRARY_PATH ${QHULL_LIBRARY} PATH)
    get_filename_component(QHULL_LIBRARY_DEBUG_PATH ${QHULL_LIBRARY_DEBUG} PATH)
    set(QHULL_LIBRARY_DIRS ${QHULL_LIBRARY_PATH} ${QHULL_LIBRARY_DEBUG_PATH}) 
  endif(QHULL_FOUND)
endmacro(find_qhull)

#remove this as soon as openni-dev is shipped with FindOpenni.cmake
macro(find_openni)
  if(NOT OPENNI_ROOT AND ("ON" STREQUAL "ON"))
    set(OPENNI_INCLUDE_DIRS "/usr/include/openni")
    set(OPENNI_LIBRARY "/usr/lib/libOpenNI.so")
  else(NOT OPENNI_ROOT AND ("ON" STREQUAL "ON"))
  if(PkgConfig_FOUND)
    pkg_check_modules(PC_OPENNI openni-dev)
  endif(PkgConfig_FOUND)
  find_path(OPENNI_INCLUDE_DIRS XnStatus.h
    HINTS ${PC_OPENNI_INCLUDEDIR} ${PC_OPENNI_INCLUDE_DIRS} 
          "${OPENNI_ROOT}" "$ENV{OPENNI_ROOT}"
    PATHS "$ENV{PROGRAMFILES}/OpenNI/Include" "$ENV{PROGRAMW6432}/OpenNI/Include"
    PATH_SUFFIXES include/openni Include)
  #add a hint so that it can find it without the pkg-config
  find_library(OPENNI_LIBRARY 
    NAMES OpenNI64 OpenNI 
    HINTS ${PC_OPENNI_LIBDIR} ${PC_OPENNI_LIBRARY_DIRS} 
          "${OPENNI_ROOT}" "$ENV{OPENNI_ROOT}"
    PATHS "$ENV{PROGRAMFILES}/OpenNI/Lib" "$ENV{PROGRAMW6432}/OpenNI/Lib64"
    PATH_SUFFIXES lib Lib Lib64)
  endif(NOT OPENNI_ROOT AND ("ON" STREQUAL "ON"))
  find_package_handle_standard_args(openni DEFAULT_MSG OPENNI_LIBRARY OPENNI_INCLUDE_DIRS)

  if(OPENNI_FOUND)
    get_filename_component(OPENNI_LIBRARY_PATH ${OPENNI_LIBRARY} PATH)
    set(OPENNI_LIBRARY_DIRS ${OPENNI_LIBRARY_PATH}) 
    set(OPENNI_LIBRARIES "${OPENNI_LIBRARY}")
  endif(OPENNI_FOUND)
endmacro(find_openni)

#remove this as soon as flann is shipped with FindFlann.cmake
macro(find_flann)
  if(PCL_ALL_IN_ONE_INSTALLER)
    set(FLANN_ROOT "${PCL_ROOT}/3rdParty/Flann")
  elseif(NOT FLANN_ROOT)
    get_filename_component(FLANN_ROOT "/usr/include" PATH)
  endif(PCL_ALL_IN_ONE_INSTALLER)
  if(PkgConfig_FOUND)
    pkg_check_modules(PC_FLANN flann)
  endif(PkgConfig_FOUND)

  find_path(FLANN_INCLUDE_DIRS flann/flann.hpp
    HINTS ${PC_FLANN_INCLUDEDIR} ${PC_FLANN_INCLUDE_DIRS} 
          "${FLANN_ROOT}" "$ENV{FLANN_ROOT}"
    PATHS "$ENV{PROGRAMFILES}/flann 1.6.9" "$ENV{PROGRAMW6432}/flann 1.6.9"
          "$ENV{PROGRAMFILES}/flann" "$ENV{PROGRAMW6432}/flann"
    PATH_SUFFIXES include)

  find_library(FLANN_LIBRARY
    NAMES flann_cpp_s flann_cpp
    HINTS ${PC_FLANN_LIBDIR} ${PC_FLANN_LIBRARY_DIRS} "${FLANN_ROOT}" "$ENV{FLANN_ROOT}"
    PATHS "$ENV{PROGRAMFILES}/flann 1.6.9" "$ENV{PROGRAMW6432}/flann 1.6.9" 
          "$ENV{PROGRAMFILES}/flann" "$ENV{PROGRAMW6432}/flann"
    PATH_SUFFIXES lib)

  find_library(FLANN_LIBRARY_DEBUG 
    NAMES flann_cpp_s-gd flann_cpp-gd flann_cpp_s flann_cpp
    HINTS ${PC_FLANN_LIBDIR} ${PC_FLANN_LIBRARY_DIRS} "${FLANN_ROOT}" "$ENV{FLANN_ROOT}"
    PATHS "$ENV{PROGRAMFILES}/flann 1.6.9" "$ENV{PROGRAMW6432}/flann 1.6.9" 
          "$ENV{PROGRAMFILES}/flann" "$ENV{PROGRAMW6432}/flann"
    PATH_SUFFIXES lib)
  if(NOT FLANN_LIBRARY_DEBUG)
    set(FLANN_LIBRARY_DEBUG ${FLANN_LIBRARY})
  endif(NOT FLANN_LIBRARY_DEBUG)

  find_package_handle_standard_args(Flann DEFAULT_MSG FLANN_LIBRARY FLANN_INCLUDE_DIRS)
  if(FLANN_FOUND)
    get_filename_component(flann_lib ${FLANN_LIBRARY} NAME_WE)
    if("${flann_lib}" STREQUAL "flann_cpp_s") # this a patch
      set(IS_FLANN_STATIC ON)
    endif("${flann_lib}" STREQUAL "flann_cpp_s")
    get_filename_component(FLANN_LIBRARY_PATH ${FLANN_LIBRARY} PATH)
    get_filename_component(FLANN_LIBRARY_DEBUG_PATH ${FLANN_LIBRARY_DEBUG} PATH)
    set(FLANN_LIBRARY_DIRS ${FLANN_LIBRARY_PATH} ${FLANN_LIBRARY_DEBUG_PATH}) 
    set(FLANN_LIBRARIES optimized ${FLANN_LIBRARY} debug ${FLANN_LIBRARY_DEBUG})
  endif(FLANN_FOUND)
endmacro(find_flann)

macro(find_VTK)
  if(PCL_ALL_IN_ONE_INSTALLER)
    set(VTK_DIR "${PCL_ROOT}/3rdParty/VTK 5.8.0/lib/vtk-5.8")
  elseif(NOT VTK_DIR)
    set(VTK_DIR "/usr/lib/vtk-5.2")
  endif(PCL_ALL_IN_ONE_INSTALLER)
  find_package(VTK ${QUIET_})
  if(VTK_FOUND)
    set(VTK_LIBRARIES vtkCommon vtkRendering vtkHybrid)
  endif(VTK_FOUND)
endmacro(find_VTK)

# Finds each component external libraries if any
# The functioning is as following
# try to find _lib
# |--> _lib found ==> include the headers,
#                     link to its library directories or include _lib_USE_FILE
# ---> _lib not found
#                   |--> _lib is optional ==> disable it (thanks to the guardians) 
#                   |                         and warn
#                   ---> _lib is required
#                                       |--> component is required explicitely ==> error
#                                       ---> component is induced ==> warn and remove it
#                                                                     from the list

macro(find_external_library _component _lib _is_optional)
  if("${_lib}" STREQUAL "boost")
    find_boost()
  elseif("${_lib}" STREQUAL "eigen")
    find_eigen()
  elseif("${_lib}" STREQUAL "flann")
    find_flann()
  elseif("${_lib}" STREQUAL "qhull")
    find_qhull()
  elseif("${_lib}" STREQUAL "openni")
    find_openni()
  elseif("${_lib}" STREQUAL "vtk")
    find_VTK()
  endif("${_lib}" STREQUAL "boost")

  string(TOUPPER "${_component}" COMPONENT)
  string(TOUPPER "${_lib}" LIB)
  if(${LIB}_FOUND)
    list(APPEND PCL_${COMPONENT}_INCLUDE_DIRS ${${LIB}_INCLUDE_DIRS})
    if(${LIB}_USE_FILE)
      include(${${LIB}_USE_FILE})
    else(${LIB}_USE_FILE)
      list(APPEND PCL_${COMPONENT}_LIBRARY_DIRS "${${LIB}_LIBRARY_DIRS}")
    endif(${LIB}_USE_FILE)
    if(${LIB}_LIBRARIES)
      list(APPEND PCL_${COMPONENT}_LIBRARIES "${${LIB}_LIBRARIES}")
    endif(${LIB}_LIBRARIES)
  else(${LIB}_FOUND)
    if("${_is_optional}" STREQUAL "OPTIONAL")
      add_definitions("-DDISABLE_${LIB}")
      pcl_message("** WARNING ** ${_component} features related to ${_lib} will be disabled")
    elseif("${_is_optional}" STREQUAL "REQUIRED")
      if((NOT PCL_FIND_ALL) OR (PCL_FIND_ALL EQUAL 1))
        pcl_report_not_found("${_component} is required but ${_lib} was not found")
      elseif(PCL_FIND_ALL EQUAL 0)
        # raise error and remove _component from PCL_TO_FIND_COMPONENTS
        string(TOUPPER "${_component}" COMPONENT)
        pcl_message("** WARNING ** ${_component} will be disabled cause ${_lib} was not found")
        list(REMOVE_ITEM PCL_TO_FIND_COMPONENTS ${_component})
      endif((NOT PCL_FIND_ALL) OR (PCL_FIND_ALL EQUAL 1))
    endif("${_is_optional}" STREQUAL "OPTIONAL")
  endif(${LIB}_FOUND)
endmacro(find_external_library)

macro(pcl_check_external_dependency _component)
endmacro(pcl_check_external_dependency)

#flatten dependencies recursivity is great \o/
macro(compute_dependencies TO_FIND_COMPONENTS)
  foreach(component ${${TO_FIND_COMPONENTS}})
    set(pcl_component pcl_${component})
    if(${pcl_component}_int_dep AND (NOT PCL_FIND_ALL))
      foreach(dependency ${${pcl_component}_int_dep})
        list(FIND ${TO_FIND_COMPONENTS} ${component} pos)
        list(FIND ${TO_FIND_COMPONENTS} ${dependency} found)
        if(found EQUAL -1)
          set(pcl_dependency pcl_${dependency})
          if(${pcl_dependency}_int_dep)
            list(INSERT ${TO_FIND_COMPONENTS} ${pos} ${dependency})
            if(pcl_${dependency}_ext_dep)
              list(APPEND pcl_${component}_ext_dep ${pcl_${dependency}_ext_dep})
            endif(pcl_${dependency}_ext_dep)
            if(pcl_${dependency}_opt_dep)
              list(APPEND pcl_${component}_opt_dep ${pcl_${dependency}_opt_dep})
            endif(pcl_${dependency}_opt_dep)
            compute_dependencies(${TO_FIND_COMPONENTS})
          else(${pcl_dependency}_int_dep)
            list(INSERT ${TO_FIND_COMPONENTS} 0 ${dependency})
          endif(${pcl_dependency}_int_dep)
        endif(found EQUAL -1)
      endforeach(dependency)
    endif(${pcl_component}_int_dep AND (NOT PCL_FIND_ALL))
  endforeach(component)
endmacro(compute_dependencies)

### ---[ Find PCL

if(PCL_FIND_QUIETLY)
  set(QUIET_ QUIET)
else(PCL_FIND_QUIETLY)
  set(QUIET_)
endif(PCL_FIND_QUIETLY)

if(WIN32)
# PCLConfig.cmake is installed to PCL_ROOT/cmake
  get_filename_component(PCL_ROOT "${PCL_DIR}" PATH)
else(WIN32)
# PCLConfig.cmake is installed to PCL_ROOT/share/pcl-x.y
  get_filename_component(PCL_ROOT "${PCL_DIR}" PATH)
  get_filename_component(PCL_ROOT "${PCL_ROOT}" PATH)
endif(WIN32)

# check whether PCLConfig.cmake is found into a PCL installation or in a build tree
if(EXISTS "${PCL_ROOT}/include/pcl-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}/pcl/pcl_config.h")
  # Found a PCL installation
  # pcl_message("Found a PCL installation")
  set(PCL_INCLUDE_DIRS "${PCL_ROOT}/include/pcl-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}")
  set(PCL_LIBRARY_DIRS "${PCL_ROOT}/lib")
  if(EXISTS "${PCL_ROOT}/3rdParty")
    set(PCL_ALL_IN_ONE_INSTALLER ON)
  endif(EXISTS "${PCL_ROOT}/3rdParty")
elseif(EXISTS "${PCL_DIR}/include/pcl/pcl_config.h")
  # Found PCLConfig.cmake in a build tree of PCL
  # pcl_message("PCL found into a build tree.")
  set(PCL_INCLUDE_DIRS "${PCL_DIR}/include") # for pcl_config.h
  set(PCL_LIBRARY_DIRS "${PCL_DIR}/lib")
  set(PCL_SOURCES_TREE "/home/carlevar/pcl-trunk")
else(EXISTS "${PCL_ROOT}/include/pcl-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}/pcl/pcl_config.h")
  pcl_report_not_found("PCL can not be found on this machine")  
endif(EXISTS "${PCL_ROOT}/include/pcl-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}/pcl/pcl_config.h")

#set a suffix for debug libraries
set(PCL_DEBUG_SUFFIX "")

set(pcl_all_components  common kdtree sample_consensus octree search range_image features registration filters io segmentation surface visualization keypoints people tracking apps )
list(LENGTH pcl_all_components PCL_NB_COMPONENTS)

#list each component dependencies IN PCL
set(pcl_kdtree_int_dep common )
set(pcl_sample_consensus_int_dep common )
set(pcl_octree_int_dep common )
set(pcl_search_int_dep common kdtree octree )
set(pcl_range_image_int_dep common )
set(pcl_features_int_dep common search kdtree octree range_image )
set(pcl_registration_int_dep common kdtree sample_consensus features )
set(pcl_filters_int_dep common sample_consensus search kdtree octree )
set(pcl_io_int_dep common octree )
set(pcl_segmentation_int_dep common search sample_consensus kdtree octree )
set(pcl_surface_int_dep common search kdtree octree )
set(pcl_visualization_int_dep common io kdtree range_image )
set(pcl_keypoints_int_dep common search kdtree octree range_image features filters )
set(pcl_people_int_dep common search kdtree )
set(pcl_tracking_int_dep common search kdtree filters octree )
set(pcl_apps_int_dep common io filters sample_consensus segmentation visualization kdtree features surface octree registration keypoints tracking search )


#list each component external dependencies (ext means mandatory and opt means optional)
set(pcl_common_ext_dep eigen boost )
set(pcl_kdtree_ext_dep flann )
set(pcl_search_ext_dep flann )
set(pcl_visualization_ext_dep vtk )


set(pcl_io_opt_dep openni vtk )
set(pcl_surface_opt_dep qhull )
set(pcl_visualization_opt_dep openni )
set(pcl_apps_opt_dep openni vtk )


set(pcl_header_only_components cuda_common)

include(FindPackageHandleStandardArgs)

#check if user provided a list of components
#if no components at all or full list is given set PCL_FIND_ALL
if(PCL_FIND_COMPONENTS)
  list(LENGTH PCL_FIND_COMPONENTS PCL_FIND_COMPONENTS_LENGTH)
  if(PCL_FIND_COMPONENTS_LENGTH EQUAL PCL_NB_COMPONENTS)
    set(PCL_TO_FIND_COMPONENTS ${pcl_all_components})
    set(PCL_FIND_ALL 1)
  else(PCL_FIND_COMPONENTS_LENGTH EQUAL PCL_NB_COMPONENTS)
    set(PCL_TO_FIND_COMPONENTS ${PCL_FIND_COMPONENTS})    
  endif(PCL_FIND_COMPONENTS_LENGTH EQUAL PCL_NB_COMPONENTS)
else(PCL_FIND_COMPONENTS)
  set(PCL_TO_FIND_COMPONENTS ${pcl_all_components})
  set(PCL_FIND_ALL 1)
endif(PCL_FIND_COMPONENTS)

compute_dependencies(PCL_TO_FIND_COMPONENTS)

# compute external dependencies per component
foreach(component ${PCL_TO_FIND_COMPONENTS})
  if(NOT PkgConfig_FOUND)
    foreach(opt ${pcl_${component}_opt_dep})
      find_external_library(${component} ${opt} OPTIONAL)
    endforeach(opt)
    foreach(ext ${pcl_${component}_ext_dep})
      find_external_library(${component} ${ext} REQUIRED)
    endforeach(ext) 
  endif(NOT PkgConfig_FOUND)
endforeach(component)

foreach(component ${PCL_TO_FIND_COMPONENTS})
  set(pcl_component pcl_${component})
  string(TOUPPER "${component}" COMPONENT)
  string(TOUPPER "pcl_${component}" PCL_COMPONENT)
  string(TOUPPER "${pcl_component}_DEFINITIONS" PCL_COMPONENT_DEFINITIONS)
  string(TOUPPER "${pcl_component}_LIBRARY_DIRS" PCL_COMPONENT_LIBRARY_DIRS)
  string(TOUPPER "${pcl_component}_LIBRARY" PCL_COMPONENT_LIBRARY)
  string(TOUPPER "${pcl_component}_LIBRARY_DEBUG" PCL_COMPONENT_LIBRARY_DEBUG)
  string(TOUPPER "${pcl_component}_LIBRARIES" PCL_COMPONENT_LIBRARIES)

  pcl_message(STATUS "looking for ${PCL_COMPONENT}")

  string(REGEX REPLACE "^cuda_(.*)$" "\\1" cuda_component "${component}")
  string(REGEX REPLACE "^gpu_(.*)$" "\\1" gpu_component "${component}")	
  
  find_path(PCL_${COMPONENT}_INCLUDE_DIR
    NAMES pcl/${component}
          pcl/cuda/${cuda_component} pcl/cuda/${gpu_component} pcl/cuda/${component}
          pcl/gpu/${cuda_component} pcl/gpu/${gpu_component} pcl/gpu/${component}
    HINTS ${PCL_INCLUDE_DIRS}
          "${PCL_SOURCES_TREE}"
    PATH_SUFFIXES
          ${component}/include
          cuda/${cuda_component}/include cuda/${gpu_component}/include
    DOC "path to ${component} headers"
    NO_DEFAULT_PATH)

  if(PCL_${COMPONENT}_INCLUDE_DIR)
    list(APPEND PCL_${COMPONENT}_INCLUDE_DIRS "${PCL_${COMPONENT}_INCLUDE_DIR}")
  else(PCL_${COMPONENT}_INCLUDE_DIR)
    #pcl_message("No include directory found for pcl_${component}.")
  endif(PCL_${COMPONENT}_INCLUDE_DIR)
  
  # Skip find_library for header only modules
  list(FIND pcl_header_only_components ${component} _is_header_only)
  if(_is_header_only EQUAL -1)
    find_library(${PCL_COMPONENT_LIBRARY} ${pcl_component}
      HINTS ${PCL_LIBRARY_DIRS}
      DOC "path to ${pcl_component} library"
      NO_DEFAULT_PATH)
    get_filename_component(${component}_library_path 
      ${${PCL_COMPONENT_LIBRARY}}
      PATH)

    find_library(${PCL_COMPONENT_LIBRARY_DEBUG} ${pcl_component}${PCL_DEBUG_SUFFIX}
      HINTS ${PCL_LIBRARY_DIRS}
      DOC "path to ${pcl_component} library debug"
      NO_DEFAULT_PATH)
    # if only release found let release be debug too
    # NOTE: this is only for non Makefile sake
    if(NOT ${PCL_COMPONENT_LIBRARY_DEBUG} AND ${PCL_COMPONENT_LIBRARY})
      set(${PCL_COMPONENT_LIBRARY_DEBUG} ${${PCL_COMPONENT_LIBRARY}})
    endif(NOT ${PCL_COMPONENT_LIBRARY_DEBUG} AND ${PCL_COMPONENT_LIBRARY})

    get_filename_component(${component}_library_path_debug 
      ${${PCL_COMPONENT_LIBRARY}_DEBUG}
      PATH)
  
    find_package_handle_standard_args(${PCL_COMPONENT} DEFAULT_MSG
      ${PCL_COMPONENT_LIBRARY} PCL_${COMPONENT}_INCLUDE_DIR)
  else(_is_header_only EQUAL -1)
    find_package_handle_standard_args(${PCL_COMPONENT} DEFAULT_MSG
      PCL_${COMPONENT}_INCLUDE_DIR)  
  endif(_is_header_only EQUAL -1)
  
  if(${PCL_COMPONENT}_FOUND)
    if(NOT "${PCL_${COMPONENT}_INCLUDE_DIRS}" STREQUAL "")
      list(REMOVE_DUPLICATES PCL_${COMPONENT}_INCLUDE_DIRS)
    endif(NOT "${PCL_${COMPONENT}_INCLUDE_DIRS}" STREQUAL "")
    list(APPEND PCL_INCLUDE_DIRS ${PCL_${COMPONENT}_INCLUDE_DIRS})
    mark_as_advanced(PCL_${COMPONENT}_INCLUDE_DIRS)
    if(_is_header_only EQUAL -1)
      list(APPEND ${PCL_COMPONENT_LIBRARIES} optimized ${${PCL_COMPONENT_LIBRARY}} debug ${${PCL_COMPONENT_LIBRARY_DEBUG}})
      list(APPEND PCL_LIBRARIES ${${PCL_COMPONENT}_LIBRARIES})
      list(APPEND PCL_LIBRARY_DIRS ${component_library_path})
      list(APPEND PCL_LIBRARY_DIRS ${component_library_path_debug})
      mark_as_advanced(${PCL_COMPONENT_LIBRARY} ${PCL_COMPONENT_LIBRARY_DEBUG})
    endif(_is_header_only EQUAL -1)    
  endif(${PCL_COMPONENT}_FOUND)
endforeach(component)

if(NOT "${PCL_INCLUDE_DIRS}" STREQUAL "")
  list(REMOVE_DUPLICATES PCL_INCLUDE_DIRS)
endif(NOT "${PCL_INCLUDE_DIRS}" STREQUAL "")

if(NOT "${PCL_LIBRARY_DIRS}" STREQUAL "")
  list(REMOVE_DUPLICATES PCL_LIBRARY_DIRS)
endif(NOT "${PCL_LIBRARY_DIRS}" STREQUAL "")

find_package_handle_standard_args(PCL DEFAULT_MSG PCL_LIBRARIES PCL_INCLUDE_DIRS)
mark_as_advanced(PCL_LIBRARIES PCL_INCLUDE_DIRS PCL_LIBRARY_DIRS)

if(PCL_FOUND)
  set(PCL_VERSION ${PCL_FOUND_VERSION})
  list(APPEND PCL_DEFINITIONS -DEIGEN_USE_NEW_STDVECTOR) 
  list(APPEND PCL_DEFINITIONS -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)
  if(WIN32)
    list(APPEND PCL_DEFINITIONS -DBOOST_ALL_NO_LIB)
    if(IS_FLANN_STATIC)
      list(APPEND PCL_DEFINITIONS -DFLANN_STATIC)
    endif(IS_FLANN_STATIC)
  endif(WIN32)
endif(PCL_FOUND)
