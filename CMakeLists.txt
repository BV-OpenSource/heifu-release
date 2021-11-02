cmake_minimum_required(VERSION 3.0.2)
project(uav_msgs)


find_package(catkin REQUIRED COMPONENTS
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  UAVWaypoint.msg
)

###################################
## catkin specific configuration ##
###################################
generate_messages(
  DEPENDENCIES
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)


###########
## Build ##
###########


