cmake_minimum_required(VERSION 2.8.3)
project(hardware_interface)

if(USE_ROSBUILD)

  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()

else()

  find_package(catkin REQUIRED COMPONENTS roscpp)

  # Declare catkin package
  catkin_package(
    CATKIN_DEPENDS roscpp
    INCLUDE_DIRS include
    )

  # Install
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

endif()
