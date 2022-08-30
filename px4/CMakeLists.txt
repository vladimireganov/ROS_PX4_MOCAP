cmake_minimum_required(VERSION 3.0.2)
project(px4)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arm
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  include
)

add_executable(my_trajectory src/test_trajectory.cpp) #code for testing
target_link_libraries(my_trajectory ${catkin_LIBRARIES})