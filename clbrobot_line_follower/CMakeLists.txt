cmake_minimum_required(VERSION 2.8.3)
project(clbrobot_line_follower)

add_definitions(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS cv_bridge roscpp rospy std_msgs genmsg geometry_msgs)
find_package(OpenCV REQUIRED)

## Declare ROS messages and services
#add_message_files()

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

catkin_package()

## Executables
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(clbrobot_line_node src/clbrobot_line.cpp src/clbrobot_line_detect.cpp)
target_link_libraries(clbrobot_line_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
#add_dependencies(clbrobot_line_node)

