cmake_minimum_required(VERSION 2.8.3)
project(controller_manager_tests)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager
  roscpp
)

catkin_python_setup()

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
    src/effort_test_controller.cpp
    src/extensible_controllers.cpp
    src/my_dummy_controller.cpp
    src/my_robot_hw.cpp
    src/pos_eff_controller.cpp
    src/pos_eff_opt_controller.cpp
    src/vel_eff_controller.cpp
  )
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

  add_executable(dummy_app EXCLUDE_FROM_ALL src/dummy_app.cpp)
  target_link_libraries(dummy_app ${PROJECT_NAME} ${catkin_LIBRARIES})

  add_executable(cm_test EXCLUDE_FROM_ALL test/cm_test.cpp)
  target_link_libraries(cm_test ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

  catkin_add_nosetests(test)

  add_rostest(test/cm_test.test DEPENDENCIES dummy_app cm_test)

  add_rostest(test/cm_msgs_utils_rostest.test)

  add_rostest(test/controller_manager_scripts.test DEPENDENCIES dummy_app)
endif()
