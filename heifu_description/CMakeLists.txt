cmake_minimum_required(VERSION 2.8.3)
project(heifu_description)

find_package(gazebo REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
    roslaunch
)

catkin_package( INCLUDE_DIRS
        ${GAZEBO_INCLUDE_DIRS}
    DEPENDS
        GAZEBO)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
)

## Add Directories
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY meshes/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
)

install(DIRECTORY models/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)

install(DIRECTORY plugins/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plugins
)

install(DIRECTORY urdf/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

install(DIRECTORY worlds/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)
