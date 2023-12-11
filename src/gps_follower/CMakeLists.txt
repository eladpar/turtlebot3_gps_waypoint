cmake_minimum_required(VERSION 3.0.2)
project(gps_follower)

## Compile as C++11, supported in ROS Kinetic and newer
## add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geographic_msgs
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  tf
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geographic_msgs geometry_msgs nav_msgs rospy sensor_msgs tf
)

## Declare an executable
add_executable(gps_follower_node
  src/GPSFollower.cpp
  src/utils.cpp
)

## Link the executable with catkin libraries
target_link_libraries(gps_follower_node
  ${catkin_LIBRARIES}
)

## Specify headers to be installed
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark executable for installation
install(TARGETS gps_follower_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
