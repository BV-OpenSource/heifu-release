cmake_minimum_required(VERSION 3.0.2)
project(priority_manager)

SET (CMAKE_C_FLAGS      "-std=c++11")
SET (CMAKE_CXX_FLAGS    "-std=c++11")


set(NEEDED_COMPONENTS
        geometry_msgs
        uav_msgs
        roscpp
        tf2
        )


find_package(catkin REQUIRED COMPONENTS
    ${NEEDED_COMPONENTS}
    )

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}_lib
    CATKIN_DEPENDS
        ${NEEDED_COMPONENTS}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(
    ${PROJECT_NAME}_node
    src/${PROJECT_NAME}_node.cpp
    src/Priority_manager.cpp
    include/${PROJECT_NAME}/Priority_manager.hpp
)

target_link_libraries(
    ${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)
