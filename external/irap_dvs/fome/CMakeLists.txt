cmake_minimum_required(VERSION 2.8.3)
project(fome)

set (CMAKE_C_CXX_COMMON_FLAGS "-Wall -Wfloat-equal -Wundef -Wno-strict-aliasing -Wno-unused-function -Wno-unused-but-set-variable -Wno-unused-macros -Wno-unknown-pragmas")
set (CMAKE_C_FLAGS "-u MAIN__ -std=gnu99 -fopenmp ${CMAKE_C_CXX_COMMON_FLAGS}")
set (CMAKE_CXX_FLAGS "-u MAIN__ -std=gnu++11 -std=c++11 -fopenmp -msse4.1 ${CMAKE_C_CXX_COMMON_FLAGS}")

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

find_package(catkin REQUIRED
	COMPONENTS
	cv_bridge
	roscpp
	rospy
	std_msgs
	image_transport
	message_generation
	sensor_msgs
	dvs_msgs
	OpenCV
)

FILE(GLOB messages_to_build RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg"
    "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")
add_message_files(
	FILES
	${messages_to_build}
)
generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS
	std_msgs
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
)

add_executable(fome_ros
	src/fome.cpp
	src/fome_ros.cpp
)
target_link_libraries(fome_ros
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)
add_dependencies(fome_ros
	${catkin_EXPORTED_TARGETS}
	${${PROJECT_NAME}_EXPORTED_TARGETS}
)


# Mark executables and/or libraries for installation
install(
	TARGETS fome_ros
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
