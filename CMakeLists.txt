cmake_minimum_required(VERSION 2.8.3)
project(rosplan_interface)

find_package(catkin REQUIRED)
catkin_package(CATKIN_DEPENDS rosplan_dispatch_msgs)
catkin_python_setup()

install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
