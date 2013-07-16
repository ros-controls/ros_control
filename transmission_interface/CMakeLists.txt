cmake_minimum_required(VERSION 2.8.3)
project(transmission_interface)

if(USE_ROSBUILD)

  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

  rosbuild_init()

  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

  # Transmission Paraser Library
  rosbuild_add_library(${PROJECT_NAME}_parser src/transmission_parser.cpp)
  target_link_libraries(${PROJECT_NAME}_parser)

  rosbuild_add_gtest(simple_transmission_test           test/simple_transmission_test.cpp)
  rosbuild_add_gtest(differential_transmission_test     test/differential_transmission_test.cpp)
  rosbuild_add_gtest(four_bar_linkage_transmission_test test/four_bar_linkage_transmission_test.cpp)
  rosbuild_add_gtest(transmission_interface_test        test/transmission_interface_test.cpp)

  # TODO: why is it explicitly needed???, without it linker fails.
  target_link_libraries(simple_transmission_test           pthread)
  target_link_libraries(differential_transmission_test     pthread)
  target_link_libraries(four_bar_linkage_transmission_test pthread)
  target_link_libraries(transmission_interface_test        pthread)

else()

  # Load catkin and all dependencies required for this package
  find_package(catkin REQUIRED hardware_interface)

  # Build
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )
  link_directories(${catkin_LIBRARY_DIRS})

  # Transmission Paraser Library
  add_library(${PROJECT_NAME}_parser
    src/transmission_parser.cpp
  )
  target_link_libraries(${PROJECT_NAME}_parser ${catkin_LIBRARIES} tinyxml)

  # Declare a catkin package
  catkin_package(
    DEPENDS tinyxml
    LIBRARIES ${PROJECT_NAME}_parser
    INCLUDE_DIRS include
  )

  # Tests
  catkin_add_gtest(simple_transmission_test           test/simple_transmission_test.cpp)
  catkin_add_gtest(differential_transmission_test     test/differential_transmission_test.cpp)
  catkin_add_gtest(four_bar_linkage_transmission_test test/four_bar_linkage_transmission_test.cpp)
  catkin_add_gtest(transmission_interface_test        test/transmission_interface_test.cpp)
  target_link_libraries(transmission_interface_test ${catkin_LIBRARIES})

  # Install
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

endif()
