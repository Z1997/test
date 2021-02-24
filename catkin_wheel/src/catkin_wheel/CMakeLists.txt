cmake_minimum_required(VERSION 2.8.3)
project(catkin_wheel)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs

)

catkin_package(
   INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs
)

include_directories( 
	${catkin_INCLUDE_DIRS}
	include
)

## Declare a C++ library
add_library(
   ${PROJECT_NAME}
   include/catkin_wheel/SH_Circle.h
   src/SH_Circle.cpp
 )


add_executable(${PROJECT_NAME}_node src/catkin_wheel.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
  )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -msse3 -std=c++11")


