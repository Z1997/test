cmake_minimum_required(VERSION 2.8.3)
project(catkin_dijstra)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES catkin_dijstra
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(
${PROJECT_NAME}
    include/catkin_dijstra/dijstra.h
    include/catkin_dijstra/blockallocator.h
    src/dijstra.cpp
    src/blockallocator.cpp
 )



add_executable(${PROJECT_NAME}_node src/catkin_dijstra.cpp)


add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
 )




set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -msse3 -std=c++11")
