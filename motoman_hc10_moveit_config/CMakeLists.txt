cmake_minimum_required(VERSION 3.1.3)
project(motoman_hc10_moveit_config)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  geometry_msgs
)

add_service_files(
  FILES
  Robot_move.srv
  Robot_move_predef.srv
  Speed_percentage.srv
)

add_message_files(
  FILES
  IHM_msg.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS 
  rospy
)

catkin_install_python(PROGRAMS
  scripts/add_interferences.py
  scripts/add_interferences_box.py
  scripts/add_interferences_simulation.py
  scripts/move_robot_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
