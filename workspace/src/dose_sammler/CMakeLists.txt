cmake_minimum_required(VERSION 3.0.2)
project(dose_sammler)

## Find catkin macros and ROS packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  message_generation
  cv_bridge
)

## Declare ROS messages
add_message_files(
  FILES
  Frontier.msg
  FrontierList.msg
)

## Generate added messages
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs nav_msgs
)

## Include directories
include_directories(${catkin_INCLUDE_DIRS})

## Install your Python executable scripts
catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/keyboard_control.py
  scripts/encoder_odometry.py
  #scripts/frontier_exploration.py
  #scripts/frontier_search.py
  #scripts/path_planner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install your Python modules from src/dose_sammler
install(DIRECTORY src/dose_sammler/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/dose_sammler
  FILES_MATCHING PATTERN "*.py"
)

