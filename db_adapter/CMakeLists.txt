cmake_minimum_required(VERSION 2.8.3)
project(db_adapter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  librarian_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES db_adapter
 CATKIN_DEPENDS roscpp rospy std_msgs 
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

