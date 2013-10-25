cmake_minimum_required(VERSION 2.8.3)
project(hardware_interface)

if(USE_ROSBUILD)

  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()

  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

  rosbuild_add_gtest(hardware_resource_manager_test     test/hardware_resource_manager_test.cpp)
  rosbuild_add_gtest(actuator_state_interface_test      test/actuator_state_interface_test.cpp)
  rosbuild_add_gtest(actuator_command_interface_test    test/actuator_command_interface_test.cpp)
  rosbuild_add_gtest(joint_state_interface_test         test/joint_state_interface_test.cpp)
  rosbuild_add_gtest(joint_command_interface_test       test/joint_command_interface_test.cpp)
  rosbuild_add_gtest(force_torque_sensor_interface_test test/force_torque_sensor_interface_test.cpp)
  rosbuild_add_gtest(imu_sensor_interface_test          test/imu_sensor_interface_test.cpp)
  rosbuild_add_gtest(robot_hw_test                      test/robot_hw_test.cpp)

  # TODO: why is it explicitly needed???, without it the linker fails.
  target_link_libraries(hardware_resource_manager_test     pthread)
  target_link_libraries(actuator_state_interface_test      pthread)
  target_link_libraries(actuator_command_interface_test    pthread)
  target_link_libraries(joint_state_interface_test         pthread)
  target_link_libraries(joint_command_interface_test       pthread)
  target_link_libraries(force_torque_sensor_interface_test pthread)
  target_link_libraries(imu_sensor_interface_test          pthread)
  target_link_libraries(robot_hw_test                      pthread)

else()

  find_package(catkin REQUIRED COMPONENTS roscpp)

  include_directories(include)

  # Declare catkin package
  catkin_package(
    CATKIN_DEPENDS roscpp
    INCLUDE_DIRS include
    )

  if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(hardware_resource_manager_test  test/hardware_resource_manager_test.cpp)
    target_link_libraries(hardware_resource_manager_test ${catkin_LIBRARIES})

    catkin_add_gtest(actuator_state_interface_test   test/actuator_state_interface_test.cpp)
    target_link_libraries(actuator_state_interface_test ${catkin_LIBRARIES})

    catkin_add_gtest(actuator_command_interface_test test/actuator_command_interface_test.cpp)
    target_link_libraries(actuator_command_interface_test ${catkin_LIBRARIES})

    catkin_add_gtest(joint_state_interface_test      test/joint_state_interface_test.cpp)
    target_link_libraries(joint_state_interface_test ${catkin_LIBRARIES})

    catkin_add_gtest(joint_command_interface_test    test/joint_command_interface_test.cpp)
    target_link_libraries(joint_command_interface_test ${catkin_LIBRARIES})

    catkin_add_gtest(force_torque_sensor_interface_test test/force_torque_sensor_interface_test.cpp)
    target_link_libraries(force_torque_sensor_interface_test ${catkin_LIBRARIES})

    catkin_add_gtest(imu_sensor_interface_test       test/imu_sensor_interface_test.cpp)
    target_link_libraries(imu_sensor_interface_test ${catkin_LIBRARIES})

    catkin_add_gtest(robot_hw_test                   test/robot_hw_test.cpp)
    target_link_libraries(robot_hw_test ${catkin_LIBRARIES})
  endif()

  # Install
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

endif()
