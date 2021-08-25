cmake_minimum_required(VERSION 2.8.3)
project(mavros_commands)

SET (CMAKE_C_FLAGS      "-std=c++11")
SET (CMAKE_CXX_FLAGS    "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
    mavros
    mavros_msgs
    roscpp
    std_msgs
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}_lib
    CATKIN_DEPENDS
        mavros
        mavros_msgs
        roscpp
        std_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(
    ${PROJECT_NAME}_node
    src/${PROJECT_NAME}_node.cpp
    src/Mavros_commands.cpp
    include/${PROJECT_NAME}/Mavros_commands.hpp
)

target_link_libraries(
    ${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)
