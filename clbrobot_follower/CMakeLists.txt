cmake_minimum_required(VERSION 2.8.3)
project(clbrobot_follower)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS  image_transport cv_bridge roscpp depth_image_proc)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp  depth_image_proc cv_bridge image_transport
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
#add_library(${PROJECT_NAME} src/follower.cpp)
add_executable(clbrobot_follower_node src/follower.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(clbrobot_follower_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_turtlebot_follower.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
