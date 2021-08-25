cmake_minimum_required(VERSION 3.0.2)
project(navigation_controller)

SET (CMAKE_C_FLAGS      "-std=c++11")
SET (CMAKE_CXX_FLAGS    "-std=c++11")

set(NEEDED_COMPONENTS
        geometry_msgs
        uav_msgs
        mavros_msgs
        roscpp
        std_msgs
        )


find_package(catkin REQUIRED COMPONENTS
    ${NEEDED_COMPONENTS}
    )

################################################
## Declare ROS messages, services and actions ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
        ${NEEDED_COMPONENTS}
    )

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

add_executable(${PROJECT_NAME}_node
    src/navigation_controller_node.cpp
    src/navigation_controller.cpp
    src/position_controller.cpp
    src/velocity_controller.cpp  
    )

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    )

