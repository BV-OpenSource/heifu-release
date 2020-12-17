cmake_minimum_required(VERSION 2.8.3)
project(heifu_battery)

SET (CMAKE_C_FLAGS      "-std=c++11")
SET (CMAKE_CXX_FLAGS    "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
    mavros
    roscpp
    mavros_msgs
    sensor_msgs
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}_lib
    CATKIN_DEPENDS
        roscpp
        mavros_msgs
        sensor_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(
    ${PROJECT_NAME}_node
    src/battery_node.cpp
    src/Battery.cpp
    include/${PROJECT_NAME}/Battery.hpp
)

target_link_libraries(
    ${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)