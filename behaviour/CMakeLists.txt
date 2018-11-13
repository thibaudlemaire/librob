cmake_minimum_required(VERSION 2.8.3)
project(behaviour)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  librarian_msgs
)

# catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES behaviour
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

