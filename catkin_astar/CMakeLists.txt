cmake_minimum_required(VERSION 2.8.3)
project(catkin_astar)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES catkin_astar
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 
  ${catkin_INCLUDE_DIRS}
  include
  
)


add_library(
   ${PROJECT_NAME}
  include/catkin_astar/Astar.h
  include/catkin_astar/blockallocator.h
  src/Astar.cpp
  src/blockallocator.cpp
 )

add_executable(${PROJECT_NAME}_node src/main.cpp)

 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
 )


add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -msse3 -std=c++11")

