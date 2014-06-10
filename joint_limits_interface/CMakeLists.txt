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

else() # catkin

  find_package(catkin REQUIRED
    roscpp
    rostest
    hardware_interface
    urdfdom
  )

  include_directories(
    SYSTEM 
    include
    ${catkin_INCLUDE_DIRS}
  )

  # Declare catkin package
  catkin_package(
    CATKIN_DEPENDS 
      roscpp
    INCLUDE_DIRS
      include
      ${catkin_INCLUDE_DIRS}
    LIBRARIES
      ${catkin_LIBRARIES}
  )

  if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(joint_limits_interface_test test/joint_limits_interface_test.cpp)
    target_link_libraries(joint_limits_interface_test ${catkin_LIBRARIES})

    catkin_add_gtest(joint_limits_urdf_test      test/joint_limits_urdf_test.cpp)
    target_link_libraries(joint_limits_urdf_test ${catkin_LIBRARIES})

    catkin_add_gtest(joint_limits_rosparam_test  test/joint_limits_urdf_test.cpp)
    target_link_libraries(joint_limits_rosparam_test ${catkin_LIBRARIES})

    add_rostest(test/joint_limits_rosparam.test)
  endif()

  # Install
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

endif()
