cmake_minimum_required(VERSION 2.8.3)
project(safety_limits_interface)

if(USE_ROSBUILD)

  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()

  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

  rosbuild_add_gtest(soft_joint_limits_interface_test  test/soft_joint_limits_interface_test.cpp)

  # TODO: why is it explicitly needed???, without it the linker fails.
  target_link_libraries(soft_joint_limits_interface_test  pthread)

else()

  find_package(catkin REQUIRED COMPONENTS roscpp)

  # Declare catkin package
  catkin_package(
    CATKIN_DEPENDS roscpp
    INCLUDE_DIRS include
    )

  catkin_add_gtest(soft_joint_limits_interface_test  test/soft_joint_limits_interface_test.cpp)

  # Install
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

endif()
