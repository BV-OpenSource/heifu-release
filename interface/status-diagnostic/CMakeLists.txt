cmake_minimum_required(VERSION 2.8.3)
project(status_diagnostic)

SET (CMAKE_C_FLAGS      "-std=c++11")
SET (CMAKE_CXX_FLAGS    "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
    diagnostic_msgs
    roscpp
    std_msgs
    topic_tools
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}_lib
    CATKIN_DEPENDS
        diagnostic_msgs
        roscpp
        std_msgs
        topic_tools
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(
    ${PROJECT_NAME}_node
    src/${PROJECT_NAME}_node.cpp
    src/Status_diagnostic.cpp
    include/${PROJECT_NAME}/Status_diagnostic.hpp
)

target_link_libraries(
    ${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)


