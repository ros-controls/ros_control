cmake_minimum_required(VERSION 3.0.2)
project(hardware_interface)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

# Declare a catkin package
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
)


###########
## Build ##
###########

# Specify header include paths
include_directories(include ${catkin_INCLUDE_DIRS})


#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(hardware_resource_manager_test           test/hardware_resource_manager_test.cpp)
  target_link_libraries(hardware_resource_manager_test      ${catkin_LIBRARIES})

  catkin_add_gtest(actuator_state_interface_test            test/actuator_state_interface_test.cpp)
  target_link_libraries(actuator_state_interface_test       ${catkin_LIBRARIES})

  catkin_add_gtest(actuator_command_interface_test          test/actuator_command_interface_test.cpp)
  target_link_libraries(actuator_command_interface_test     ${catkin_LIBRARIES})

  catkin_add_gtest(joint_state_interface_test               test/joint_state_interface_test.cpp)
  target_link_libraries(joint_state_interface_test          ${catkin_LIBRARIES})

  catkin_add_gtest(joint_command_interface_test             test/joint_command_interface_test.cpp)
  target_link_libraries(joint_command_interface_test        ${catkin_LIBRARIES})

  catkin_add_gtest(force_torque_sensor_interface_test       test/force_torque_sensor_interface_test.cpp)
  target_link_libraries(force_torque_sensor_interface_test  ${catkin_LIBRARIES})

  catkin_add_gtest(imu_sensor_interface_test                test/imu_sensor_interface_test.cpp)
  target_link_libraries(imu_sensor_interface_test           ${catkin_LIBRARIES})

  catkin_add_gtest(robot_hw_test                            test/robot_hw_test.cpp)
  target_link_libraries(robot_hw_test                       ${catkin_LIBRARIES})

  catkin_add_gtest(interface_manager_test                   test/interface_manager_test.cpp)
  target_link_libraries(interface_manager_test              ${catkin_LIBRARIES})

  catkin_add_gtest(posvel_command_interface_test            test/posvel_command_interface_test.cpp)
  target_link_libraries(posvel_command_interface_test       ${catkin_LIBRARIES})

  catkin_add_gtest(posvelacc_command_interface_test         test/posvelacc_command_interface_test.cpp)
  target_link_libraries(posvelacc_command_interface_test    ${catkin_LIBRARIES})
endif()


#############
## Install ##
#############

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
