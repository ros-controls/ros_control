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

  include_directories(include)
  include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

  catkin_add_gmock(controller_base_test
    test/controller_base_test.cpp
  )

  target_link_libraries(controller_base_test
    ${catkin_LIBRARIES}
  )

endif()

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

