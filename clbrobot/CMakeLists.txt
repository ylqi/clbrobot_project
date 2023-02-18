cmake_minimum_required(VERSION 2.8.3)
project(clbrobot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  rosserial_python
  sensor_msgs
  clb_msgs
  std_msgs
  tf
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES clbrobot
  CATKIN_DEPENDS roscpp rospy tf nav_msgs clb_msgs geometry_msgs sensor_msgs std_msgs 
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(clb_base src/clb_base.cpp)
add_executable(clb_base_node src/clb_base_node.cpp)
target_link_libraries(clb_base_node clb_base ${catkin_LIBRARIES})

