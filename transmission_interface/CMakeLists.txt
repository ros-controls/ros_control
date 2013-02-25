cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
# set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_gtest(simple_transmission_test           test/simple_transmission_test.cpp)
rosbuild_add_gtest(differential_transmission_test     test/differential_transmission_test.cpp)
rosbuild_add_gtest(four_bar_linkage_transmission_test test/four_bar_linkage_transmission_test.cpp)
rosbuild_add_gtest(transmission_interface_test        test/transmission_interface_test.cpp)

# TODO: why is it explicitly needed???, without it linker fails.
target_link_libraries(simple_transmission_test           pthread)
target_link_libraries(differential_transmission_test     pthread)
target_link_libraries(four_bar_linkage_transmission_test pthread)
target_link_libraries(transmission_interface_test        pthread)
