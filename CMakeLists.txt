cmake_minimum_required(VERSION 2.4.6 FATAL_ERROR)
project(wintracker)

find_package(catkin REQUIRED COMPONENTS
	cmake_modules
	roscpp
	std_msgs
	geometry_msgs
	gazebo_msgs
	std_srvs
	tf
	tf_conversions
	eigen_conversions	
	message_generation
)

find_package(Eigen3 REQUIRED)

MESSAGE("${EIGEN3_INCLUDE_DIR} are the directories")

add_service_files(
	FILES
	GetPose.srv
        Synchronize.srv   
)

generate_messages(
  DEPENDENCIES
	geometry_msgs
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES WTracker
	CATKIN_DEPENDS roscpp std_msgs geometry_msgs gazebo_msgs std_srvs tf message_runtime cmake_modules eigen_conversions tf_conversions	
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)

add_library(WTracker src/WTracker.c)
target_link_libraries(WTracker usb)

add_executable(wintracker_node src/wintracker_node.cpp src/wintracker_publisher.cpp)
target_link_libraries(wintracker_node WTracker ${catkin_LIBRARIES})
add_dependencies(wintracker_node wintracker_gencpp)

add_executable(wintracker_rebase src/WinRebase.cc)
target_link_libraries(wintracker_rebase ${catkin_LIBRARIES})
