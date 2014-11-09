cmake_minimum_required(VERSION 2.8.3)
project(composite_hardware_interface)

find_package(catkin REQUIRED COMPONENTS roscpp hardware_interface pluginlib)

include_directories(include)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp hardware_interface pluginlib urdf
  INCLUDE_DIRS include
) 

# Install
install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

