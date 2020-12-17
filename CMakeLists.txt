cmake_minimum_required(VERSION 2.8.3)
project(heifu_simple_waypoint)

SET (CMAKE_C_FLAGS      "-std=c++11")
SET (CMAKE_CXX_FLAGS    "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
    mavros
    roscpp
    roslib
    ros_utils
    sensor_msgs
    std_msgs
    tf
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}_lib
    CATKIN_DEPENDS
        mavros
        roscpp
        roslib
        ros_utils
        sensor_msgs
        std_msgs
        tf
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(
    ${PROJECT_NAME}_node
    src/${PROJECT_NAME}_node.cpp
    src/Heifu_simple_waypoint.cpp
    include/${PROJECT_NAME}/Heifu_simple_waypoint.hpp
)

target_link_libraries(
    ${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)

