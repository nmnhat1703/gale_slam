cmake_minimum_required(VERSION 3.0.2)
project(record)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/fisheye_recorder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
