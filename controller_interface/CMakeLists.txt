cmake_minimum_required(VERSION 2.8.3)
project(controller_interface)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp hardware_interface pluginlib)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp hardware_interface pluginlib
  INCLUDE_DIRS include
  )

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  include_directories(include ${catkin_INCLUDE_DIRS})
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11")
  catkin_add_gtest(composite_controller_test test/composite_controller_test.cpp)
endif()
