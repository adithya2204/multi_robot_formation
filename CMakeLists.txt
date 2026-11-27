cmake_minimum_required(VERSION 3.0.2)
project(multi_robot_formation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/triangle_formation.py
  scripts/square_formation.py
  scripts/pentagon_formation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
