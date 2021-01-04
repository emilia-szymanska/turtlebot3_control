cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES turtlebot3_control
#  CATKIN_DEPENDS gazebo_ros rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

