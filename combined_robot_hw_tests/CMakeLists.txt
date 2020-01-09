cmake_minimum_required(VERSION 2.8.3)
project(combined_robot_hw_tests)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  combined_robot_hw
  controller_manager
  controller_manager_tests
  hardware_interface
  roscpp
)

# Declare a catkin package
catkin_package()


#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # Specify header include paths
  include_directories(include ${catkin_INCLUDE_DIRS})

  add_library(${PROJECT_NAME} EXCLUDE_FROM_ALL
    src/my_robot_hw_1.cpp
    src/my_robot_hw_2.cpp
    src/my_robot_hw_3.cpp
    src/my_robot_hw_4.cpp
  )
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

  add_executable(combined_robot_hw_dummy_app EXCLUDE_FROM_ALL src/dummy_app.cpp)
  target_link_libraries(combined_robot_hw_dummy_app ${PROJECT_NAME} ${catkin_LIBRARIES})

  add_rostest_gtest(combined_robot_hw_test
    test/combined_robot_hw_test.test
    test/combined_robot_hw_test.cpp
  )
  target_link_libraries(combined_robot_hw_test ${PROJECT_NAME} ${catkin_LIBRARIES})

  add_rostest_gtest(combined_robot_hw_cm_test
    test/cm_test.test
    test/cm_test.cpp
  )
  add_dependencies(combined_robot_hw_cm_test combined_robot_hw_dummy_app)
  target_link_libraries(combined_robot_hw_cm_test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()
