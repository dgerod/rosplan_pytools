cmake_minimum_required(VERSION 2.8.3)
project(rosplan_pytools)

find_package(catkin REQUIRED)
catkin_package(CATKIN_DEPENDS
    rospy std_msgs mongodb_store
    rosplan_dispatch_msgs rosplan_knowlege_msgs)

catkin_python_setup()
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rospy std_msgs mongodb_store
                 rosplan_dispatch_msgs rosplan_knowlege_msgs
  DEPENDS
)

install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
