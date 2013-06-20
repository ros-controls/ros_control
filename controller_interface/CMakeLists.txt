cmake_minimum_required(VERSION 2.8.3)
project(controller_interface)

if(USE_ROSBUILD)

  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

  rosbuild_init()

else()

  # Load catkin and all dependencies required for this package
  find_package(catkin REQUIRED COMPONENTS roscpp hardware_interface pluginlib)

  # Declare catkin package
  catkin_package(
    DEPENDS boost
    CATKIN_DEPENDS roscpp hardware_interface pluginlib
    INCLUDE_DIRS include
    )

  # Install
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

endif()
