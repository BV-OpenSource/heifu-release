cmake_minimum_required(VERSION 2.8.3)
project(gcs_interface)

find_package(catkin REQUIRED COMPONENTS
    rospy
    geographic_msgs
    geometry_msgs
    mavros_msgs
    std_msgs
)

#catkin_package(
#    INCLUDE_DIRS
#        include
#    LIBRARIES
#        ${PROJECT_NAME}_lib
#    CATKIN_DEPENDS
#        rospy
#)

catkin_python_setup()

#include_directories(
#    include
#    ${catkin_INCLUDE_DIRS}
#)

#add_executable(
#    ${PROJECT_NAME}_node
#    src/${PROJECT_NAME}_node.cpp
#    src/Mavros_commands.cpp
#    include/${PROJECT_NAME}/Mavros_commands.hpp
#)

#target_link_libraries(
#    ${PROJECT_NAME}_node
#    ${catkin_LIBRARIES}
#)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simontest
  CATKIN_DEPENDS rospy geographic_msgs geometry_msgs mavros_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
    config
    ${catkin_INCLUDE_DIRS}
)

#catkin_install_python(PROGRAMS
#    modules/interface.py
#    modules/heifu_interface.py
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
