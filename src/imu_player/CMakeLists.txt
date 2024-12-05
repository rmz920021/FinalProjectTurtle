cmake_minimum_required(VERSION 3.0.2)
project(imu_player)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)

## Declare a catkin package
catkin_package()

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)
