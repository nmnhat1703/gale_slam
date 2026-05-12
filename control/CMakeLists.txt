cmake_minimum_required(VERSION 3.0.2)
project(control)


find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  rospy
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES control
#  CATKIN_DEPENDS nav_msgs rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/publish_pose.py
  src/slam_pose.py
  src/fake_gps_set_home.py
  src/planner2px4.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

