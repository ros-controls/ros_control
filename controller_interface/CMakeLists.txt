cmake_minimum_required(VERSION 2.8.3)
project(controller_interface)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp hardware_interface pluginlib)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp hardware_interface pluginlib
  INCLUDE_DIRS include
  )

if(CATKIN_ENABLE_TESTING)

  find_package(rosunit REQUIRED)

  catkin_add_gmock(controller_base_test
    test/controller_base_test.cpp
  )

  target_include_directories(controller_base_test PRIVATE include)
  target_include_directories(controller_base_test SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})

  target_link_libraries(controller_base_test
    ${catkin_LIBRARIES}
  )

endif()

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

