cmake_minimum_required(VERSION 2.8.3)
project(controller_manager_tests)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS controller_manager controller_interface control_toolbox)
catkin_python_setup()

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS controller_manager controller_interface control_toolbox
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  )

#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME}
  src/my_robot_hw.cpp
  include/controller_manager_tests/my_robot_hw.h
  src/effort_test_controller.cpp
  include/controller_manager_tests/effort_test_controller.h
  src/vel_eff_controller.cpp
  include/controller_manager_tests/vel_eff_controller.h
  src/pos_eff_controller.cpp
  include/controller_manager_tests/pos_eff_controller.h
  src/pos_eff_opt_controller.cpp
  include/controller_manager_tests/pos_eff_opt_controller.h
  src/my_dummy_controller.cpp
  include/controller_manager_tests/my_dummy_controller.h
  )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(dummy_app src/dummy_app.cpp)
target_link_libraries(dummy_app ${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest)
  add_executable(cm_test test/cm_test.cpp)
  add_dependencies(tests cm_test)
  target_link_libraries(cm_test ${GTEST_LIBRARIES} ${catkin_LIBRARIES})
  add_rostest(test/cm_test.test)
  catkin_add_nosetests(test)
  add_rostest(test/cm_msgs_utils_rostest.test)
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

