cmake_minimum_required(VERSION 2.8.3)
project(controller_manager_tests)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS controller_manager controller_interface)
catkin_python_setup()

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS controller_manager controller_interface
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  )

#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME}
  src/effort_test_controller.cpp
  src/my_dummy_controller.cpp
  src/my_robot_hw.cpp
  src/pos_eff_controller.cpp
  src/pos_eff_opt_controller.cpp
  src/vel_eff_controller.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(dummy_app src/dummy_app.cpp)
target_link_libraries(dummy_app ${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_executable(cm_test test/cm_test.cpp)
  add_dependencies(tests cm_test)
  target_link_libraries(cm_test ${GTEST_LIBRARIES} ${catkin_LIBRARIES})
  add_rostest(test/cm_test.test)
  catkin_add_nosetests(test)
  add_rostest(test/cm_msgs_utils_rostest.test)
  add_rostest(test/controller_manager_scripts.test)
endif()

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME} dummy_app
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES test_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
