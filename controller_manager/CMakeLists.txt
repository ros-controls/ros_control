cmake_minimum_required(VERSION 2.8.3)
project(controller_manager)

if(USE_ROSBUILD)

  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

  rosbuild_init()

  rosbuild_add_library(${PROJECT_NAME} 
    src/controller_manager.cpp
    include/controller_manager/controller_manager.h
    include/controller_manager/controller_loader_interface.h
    include/controller_manager/controller_loader.h)

else()

  # Load catkin and all dependencies required for this package
  find_package(catkin REQUIRED COMPONENTS controller_interface controller_manager_msgs hardware_interface realtime_tools pluginlib)

  include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

  add_library(${PROJECT_NAME}
    src/controller_manager.cpp
    include/controller_manager/controller_manager.h
    include/controller_manager/controller_loader_interface.h
    include/controller_manager/controller_loader.h)

  # Declare catkin package
  catkin_package(
    DEPENDS boost
    CATKIN_DEPENDS controller_interface controller_manager_msgs hardware_interface realtime_tools pluginlib
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    )

endif()
