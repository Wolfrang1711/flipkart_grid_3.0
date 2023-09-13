cmake_minimum_required(VERSION 3.0.2)
project(grid_arena)


find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  rospy
  roscpp
  std_msgs
  message_generation
)

find_package(
  OpenCV REQUIRED
)

add_message_files(
  FILES
  rpm.msg
)

generate_dynamic_reconfigure_options(
  cfg/pid.cfg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/image_feed.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/path_plan.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
