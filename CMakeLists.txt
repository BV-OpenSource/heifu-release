cmake_minimum_required(VERSION 2.8.3)
project(gimbal)

SET (CMAKE_C_FLAGS      "-std=c++11")
SET (CMAKE_CXX_FLAGS    "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        mavros_msgs
        std_msgs
        sensor_msgs
        message_generation
)

add_message_files(
        FILES
        getGimbalAxes.msg
        setGimbalAxes.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}_lib
        CATKIN_DEPENDS roscpp rospy mavros_msgs std_msgs sensor_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(
        ${PROJECT_NAME}_node
        src/${PROJECT_NAME}_node.cpp
        src/Gimbal.cpp
        include/${PROJECT_NAME}/Gimbal.hpp
)

target_link_libraries(
        ${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
)

add_dependencies(
        ${PROJECT_NAME}_node
        ${PROJECT_NAME}_generate_messages_cpp
)
