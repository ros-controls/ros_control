cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_gtest(hardware_resource_manager_test  test/hardware_resource_manager_test.cpp)
rosbuild_add_gtest(actuator_state_interface_test   test/actuator_state_interface_test.cpp)
rosbuild_add_gtest(actuator_command_interface_test test/actuator_command_interface_test.cpp)
rosbuild_add_gtest(joint_state_interface_test      test/joint_state_interface_test.cpp)
rosbuild_add_gtest(joint_command_interface_test    test/joint_command_interface_test.cpp)
rosbuild_add_gtest(robot_hw_test                   test/robot_hw_test.cpp)

# TODO: why is it explicitly needed???, without it the linker fails.
target_link_libraries(hardware_resource_manager_test  pthread)
target_link_libraries(actuator_state_interface_test   pthread)
target_link_libraries(actuator_command_interface_test pthread)
target_link_libraries(joint_state_interface_test      pthread)
target_link_libraries(joint_command_interface_test    pthread)
target_link_libraries(robot_hw_test                   pthread)
