cmake_minimum_required(VERSION 2.4.6 FATAL_ERROR)
project(wintracker)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	geometry_msgs
	gazebo_msgs
	std_srvs
	tf
	message_generation
)

set(Eigen3_INCLUDE_DIR "$ENV{Eigen3_INCLUDE_DIR}")
IF(NOT EIGEN3_INCLUDE_DIR)
    MESSAGE(FATAL_ERROR "Please set the environment variable Eigen3_INCLUDE_DIR to the include directory of your Eigen3 installation. This is usually '/usr/include/eigen3'.")
ENDIF()

add_service_files(
	FILES
	GetPose.srv
)

generate_messages(
  DEPENDENCIES
	geometry_msgs
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES WTracker
	CATKIN_DEPENDS roscpp std_msgs geometry_msgs gazebo_msgs std_srvs	tf message_runtime
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${Eigen3_INCLUDE_DIR}
)

add_library(WTracker src/WTracker.c)
target_link_libraries(WTracker usb)

add_executable(wintracker_node src/wintracker_node.cpp src/wintracker_publisher.cpp)
target_link_libraries(wintracker_node WTracker ${catkin_LIBRARIES})
add_dependencies(wintracker_node wintracker_gencpp)
