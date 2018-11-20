cmake_minimum_required(VERSION 2.8.3)
project(hardware_interface)

find_package(catkin REQUIRED COMPONENTS roscpp)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp
  INCLUDE_DIRS include
  )

if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)
  include_directories(${catkin_INCLUDE_DIRS} include)

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

  catkin_add_gtest(interface_manager_test          test/interface_manager_test.cpp)
  target_link_libraries(interface_manager_test ${catkin_LIBRARIES})

  catkin_add_gtest(posvel_command_interface_test test/posvel_command_interface_test.cpp)
  target_link_libraries(posvel_command_interface_test ${catkin_LIBRARIES})

  catkin_add_gtest(posvelacc_command_interface_test test/posvelacc_command_interface_test.cpp)
  target_link_libraries(posvelacc_command_interface_test ${catkin_LIBRARIES})

endif()

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
