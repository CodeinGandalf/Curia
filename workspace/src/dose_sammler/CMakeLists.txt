cmake_minimum_required(VERSION 3.0.2)
project(dose_sammler)

## Find catkin macros and ROS packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)

## Declare catkin package
catkin_package()

## Install your Python executable scripts
catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/keyboard_control.py
  scripts/encoder_odometry.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install your Python modules from src/dose_sammler
install(DIRECTORY src/dose_sammler/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/dose_sammler
  FILES_MATCHING PATTERN "*.py"
)

