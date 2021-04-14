cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3burger)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation message_runtime)

## Declare ROS messages and services
add_message_files(
        FILES 
        CustomSound.msg
        EulerAngles.msg
        ScanAnalyzed.msg
        Movement.msg
)

add_service_files(
	FILES 
	SetThreshold.srv
	SetSpeed.srv
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()


set(MY_INCLUDE_DIRS
#	"include/"
	"/usr/local/include/"
	)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${MY_INCLUDE_DIRS})

link_directories("/usr/local/lib/")


#add_executable(key_controller src/key_controller.cpp)
#target_link_libraries(key_controller ${catkin_LIBRARIES})
#add_dependencies(key_controller pirosbot_generate_messages_cpp)
