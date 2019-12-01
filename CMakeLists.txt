cmake_minimum_required(VERSION 2.8.3)
project(metis)

find_package(catkin REQUIRED COMPONENTS
  rospy
  uav_msgs
  rosplane
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy uav_msgs rosplane
)

include_directories(include)
include_directories(
   ${catkin_INCLUDE_DIRS}
)
