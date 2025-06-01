cmake_minimum_required(VERSION 3.0.2)
project(dose_sammler)

## Find catkin macros and ROS packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
)

## Declare catkin package
catkin_package()

## Include directories
include_directories(${catkin_INCLUDE_DIRS})

## Add the C++ executable for encoder_odometry
add_executable(encoder_odometry scripts/encoder_odometry.cpp)
target_link_libraries(encoder_odometry ${catkin_LIBRARIES} wiringPi)

## Install your Python executable scripts
catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/keyboard_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install your Python modules from src/dose_sammler
install(DIRECTORY src/dose_sammler/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/dose_sammler
  FILES_MATCHING PATTERN "*.py"
)

