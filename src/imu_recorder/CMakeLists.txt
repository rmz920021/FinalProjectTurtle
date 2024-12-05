cmake_minimum_required(VERSION 3.0.2)
project(imu_recorder)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)

## Declare a catkin package
catkin_package()

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

