cmake_minimum_required(VERSION 2.8.3)
project(realtime_tools)

if(USE_ROSBUILD)

  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()
  rosbuild_add_boost_directories()
  rosbuild_add_library(${PROJECT_NAME} src/realtime_clock.cpp)

else()

  # Load catkin and all dependencies required for this package
  find_package(catkin REQUIRED COMPONENTS roscpp rospy)

  include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
  add_library(${PROJECT_NAME} src/realtime_clock.cpp)

  # Declare catkin package
  catkin_package(
    DEPENDS boost
    CATKIN_DEPENDS roscpp rospy
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    )

  # Install
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

  install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

endif()
