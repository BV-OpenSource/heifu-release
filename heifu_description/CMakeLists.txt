cmake_minimum_required(VERSION 2.8.3)
project(heifu_description)

find_package(catkin REQUIRED COMPONENTS
    gazebo_dev
    roslaunch
)

catkin_package( INCLUDE_DIRS
                    ${gazebo_dev_INCLUDE_DIRS}
                CATKIN_DEPENDS
                    gazebo_dev
                    roslaunch
                    )

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${gazebo_dev_INCLUDE_DIRS}
)

link_directories(
    ${gazebo_dev_LIBRARY_DIRS}
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
