cmake_minimum_required(VERSION 2.8.3)
set(PROJECT_NAME samsungdvs)
project(${PROJECT_NAME})

set (CMAKE_C_CXX_COMMON_FLAGS "-Wall -Wfloat-equal -Wundef -Wno-strict-aliasing -Wno-unused-function -Wno-unused-but-set-variable -Wno-unused-macros -Wno-unknown-pragmas")
set (CMAKE_C_FLAGS "-u MAIN__ -std=gnu99 -fopenmp ${CMAKE_C_CXX_COMMON_FLAGS}")
set (CMAKE_CXX_FLAGS "-u MAIN__ -std=gnu++11 -std=c++11 -fopenmp -msse4.1 ${CMAKE_C_CXX_COMMON_FLAGS}")
set (CMAKE_VERBOSE_MAKEFILE true )

find_package(OpenCV)
find_package(catkin REQUIRED
	COMPONENTS
	nodelet
	roscpp
	rospy
	std_msgs
	sensor_msgs
	dvs_msgs
	cv_bridge
	image_transport
	genmsg
)

set (opencv_DIR "/opt/ros/kinetic/include/opencv-3.2.0-dev/opencv")
set (CYPRESS_INC_PATH "/usr/local/include/")
set (CYPRESS_LIB_PATH "/usr/local/lib/libcyusb.so")

include_directories(
	include
	${CYPRESS_INC_PATH}
	${catkin_INCLUDE_DIRS}
	${PROJECT_NAME}/src
	${PROJECT_NAME}/config
)
link_directories(${PROJECT_NAME}/src)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS
	roscpp
	nodelet
	dvs_msgs
	cv_bridge
	image_transport
	message_runtime
	DEPENDS OpenCV
)

add_executable(dvsbulk
	src/dvsbulk.cpp
)
target_link_libraries(dvsbulk
	${CYPRESS_LIB_PATH}
	${CYPRESS_INC_PATH}
	${OpenCV_LIBRARIES}
	/usr/lib/x86_64-linux-gnu/libpthread.so
)


add_executable(dvsview
	src/dvsview.cpp
)
target_link_libraries(dvsview
	${CYPRESS_LIB_PATH}
	${CYPRESS_INC_PATH}
	${OpenCV_LIBRARIES}
	/usr/local/lib/libopencv_core.so
	/usr/local/lib/libopencv_highgui.so
)

add_executable(publishdvs
	src/publishdvs.cpp
)
target_link_libraries(publishdvs
	${CYPRESS_LIB_PATH}
	${CYPRESS_INC_PATH}
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)

install(
	TARGETS dvsbulk dvsview publishdvs
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
