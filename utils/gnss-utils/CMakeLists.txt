cmake_minimum_required(VERSION 2.8.3)
project(gnss_utils)

SET (CMAKE_C_FLAGS      "-std=c++11")
SET (CMAKE_CXX_FLAGS    "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    geographic_msgs
    roscpp
    rospy
    std_msgs
    mavros_msgs
    std_srvs
    )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES gnss_utils
    CATKIN_DEPENDS roscpp rospy std_msgs mavros_msgs geographic_msgs std_srvs
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

# Declare a C++ library
add_library(
    ${PROJECT_NAME}
    src/${PROJECT_NAME}/GNSS_utils.cpp
    )

add_executable(LLA2ENU_node
    src/${PROJECT_NAME}/LLA2ENU.cpp
    src/${PROJECT_NAME}/LLA2ENU_node.cpp
    src/${PROJECT_NAME}/GNSS_utils.cpp
    )

target_link_libraries(LLA2ENU_node
    ${catkin_LIBRARIES}
    )

add_dependencies(LLA2ENU_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(LLA2ENU_MavROS_node
    src/${PROJECT_NAME}/LLA2ENU_MavROS.cpp
    src/${PROJECT_NAME}/LLA2ENU_MavROS_node.cpp
    src/${PROJECT_NAME}/GNSS_utils.cpp
    )

target_link_libraries(LLA2ENU_MavROS_node
    ${catkin_LIBRARIES}
    )

add_dependencies(LLA2ENU_MavROS_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    )

