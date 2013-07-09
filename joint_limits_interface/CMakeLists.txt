cmake_minimum_required(VERSION 2.8.3)
project(joint_limits_interface)

if(USE_ROSBUILD)

  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()

  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

  rosbuild_add_gtest(joint_limits_interface_test test/joint_limits_interface_test.cpp)
  rosbuild_add_gtest(joint_limits_urdf_test      test/joint_limits_urdf_test.cpp)

  rosbuild_add_executable(joint_limits_rosparam_test test/joint_limits_rosparam_test.cpp)
  rosbuild_add_gtest_build_flags(joint_limits_rosparam_test)
  rosbuild_add_rostest(test/joint_limits_rosparam.test)

  # TODO: why is it explicitly needed???, without it the linker fails.
  target_link_libraries(joint_limits_interface_test pthread)
  target_link_libraries(joint_limits_urdf_test      pthread)
  target_link_libraries(joint_limits_rosparam_test  pthread)

else()

  find_package(catkin REQUIRED COMPONENTS roscpp rostest)

  # Declare catkin package
  catkin_package(
    CATKIN_DEPENDS roscpp
    INCLUDE_DIRS include
    )

  catkin_add_gtest(joint_limits_interface_test test/joint_limits_interface_test.cpp)
  catkin_add_gtest(joint_limits_urdf_test      test/joint_limits_urdf_test.cpp)
  catkin_add_gtest(joint_limits_rosparam_test  test/joint_limits_urdf_test.cpp)
  add_rostest(test/joint_limits_rosparam.test)

  # Install
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

endif()
