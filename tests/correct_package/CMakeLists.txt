cmake_minimum_required(VERSION 3.0.2)
project(correct_package)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  trajectory_msgs
)

catkin_package()

catkin_install_python(PROGRAMS src/pick_place_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  package.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)