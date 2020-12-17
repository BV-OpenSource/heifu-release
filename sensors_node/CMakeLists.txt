cmake_minimum_required(VERSION 2.8.3)
project(sensors_node)



find_package(catkin REQUIRED COMPONENTS

	rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  rfid_humtemp.msg
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime

)

catkin_install_python(PROGRAMS
        src/sensors_node.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


