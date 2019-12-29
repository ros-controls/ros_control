cmake_minimum_required(VERSION 2.8.3)
project(controller_manager_tests)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager
)

catkin_python_setup()

# Declare a catkin package
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    controller_interface
    controller_manager
)


###########
## Build ##
###########

# Specify header include paths
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/effort_test_controller.cpp
  src/extensible_controllers.cpp
  src/my_dummy_controller.cpp
  src/my_robot_hw.cpp
  src/pos_eff_controller.cpp
  src/pos_eff_opt_controller.cpp
  src/vel_eff_controller.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(dummy_app src/dummy_app.cpp)
target_link_libraries(dummy_app ${PROJECT_NAME} ${catkin_LIBRARIES})


#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_executable(cm_test test/cm_test.cpp)
  add_dependencies(tests cm_test)
  target_link_libraries(cm_test ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

  add_rostest(test/cm_test.test)

  catkin_add_nosetests(test)

  add_rostest(test/cm_msgs_utils_rostest.test)

  add_rostest(test/controller_manager_scripts.test)
endif()


#############
## Install ##
#############

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install targets
install(TARGETS ${PROJECT_NAME} dummy_app
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install plugins
install(FILES test_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
