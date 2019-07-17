cmake_minimum_required(VERSION 2.8.3)
project(rosplan_pytools)

find_package(catkin REQUIRED COMPONENTS
    rospy genmsg message_generation actionlib_msgs std_msgs
    rosplan_dispatch_msgs rosplan_knowledge_msgs
)

add_service_files(
    FILES
    AddElement.srv
    DiagnosticsDB.srv
    FindElement.srv
    RemoveElement.srv
    ResetDB.srv
    RetrieveElements.srv
    UpdateElement.srv
)

catkin_python_setup()

generate_messages(DEPENDENCIES
    std_msgs rosplan_dispatch_msgs rosplan_knowledge_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rospy genmsg message_generation message_runtime std_msgs
                 rosplan_dispatch_msgs rosplan_knowledge_msgs
)

install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
