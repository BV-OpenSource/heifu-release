cmake_minimum_required(VERSION 2.8.3)
project(heifu_safety)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  ros_utils
  geometry_msgs
  heifu_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS roscpp rospy ros_utils geometry_msgs heifu_msgs nav_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

add_executable(
    ${PROJECT_NAME}_node
    src/${PROJECT_NAME}_node.cpp
    src/Heifu_safety.cpp
    include/${PROJECT_NAME}/Heifu_safety.hpp
    )

target_link_libraries(
    ${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    )

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)


