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

rosbuild_add_gtest(simple_transmission_test           test/simple_transmission_test.cpp)
rosbuild_add_gtest(differential_transmission_test     test/differential_transmission_test.cpp)
rosbuild_add_gtest(four_bar_linkage_transmission_test test/four_bar_linkage_transmission_test.cpp)