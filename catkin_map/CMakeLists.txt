cmake_minimum_required(VERSION 2.8.3)
project(catkin_map)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES catkin_map
CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


 add_executable(${PROJECT_NAME}_node src/getPoint.cpp)


## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
 )


## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -msse3 -std=c++11")
