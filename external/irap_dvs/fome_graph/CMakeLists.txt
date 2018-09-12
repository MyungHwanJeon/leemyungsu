cmake_minimum_required(VERSION 2.8.3)
project(fome_graph)
set(POD_NAME fome_graph)

cmake_policy (VERSION 2.8)
cmake_policy (SET CMP0002 OLD)
set (CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE)
set (FIND_LIBRARY_USE_LIB64_PATHS 1)
set (CMAKE_C_CXX_COMMON_FLAGS "-Wall -Wfloat-equal -Wundef -Wno-strict-aliasing -Wno-unused-function -Wno-unused-but-set-variable -Wno-unused-macros -Wno-unknown-pragmas")
set (CMAKE_C_FLAGS "-u MAIN__ -std=gnu99 -fopenmp ${CMAKE_C_CXX_COMMON_FLAGS}")
set (CMAKE_CXX_FLAGS "-u MAIN__ -std=gnu++11 -std=c++11 -fopenmp -msse4.1 ${CMAKE_C_CXX_COMMON_FLAGS}")

if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
    "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
endif ()
# INCLUDE LOCAL MODULES
#========================================================================
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "./cmake")
file (GLOB MODULES "${CMAKE_MODULE_PATH}/*.cmake")
foreach (MODULE ${MODULES})
  include (${MODULE})
endforeach ()

find_package(cmake_modules REQUIRED)
find_package(fome REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

libhandler_eigen3()
find_package(catkin REQUIRED
	COMPONENTS
	cv_bridge
	image_transport
	message_generation
	sensor_msgs
	dvs_msgs
	OpenCV
	rosbag
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS
	dvs_msgs
	cv_bridge
	image_transport
	message_runtime
	sensor_msgs
)
#add executables

add_definitions(-std=c++11)

include_directories(
	include
	${PROJECT_NAME}/src
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${fome_INCLUDE_DIRS}
)

add_executable(fome_graph
	src/fome_graph.cpp
)

target_link_libraries(fome_graph
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${PROJECT_SOURCE_DIR}/third-party/g2o/lib/libg2o.so
)
add_dependencies(fome_graph
	${catkin_EXPORTED_TARGETS}
	${${PROJECT_NAME}_EXPORTED_TARGETS}
)


# Mark executables and/or libraries for installation
install(
	TARGETS fome_graph
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
	DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)
