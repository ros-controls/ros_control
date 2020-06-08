cmake_minimum_required(VERSION 3.0.2)
project(controller_manager_tests)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager
  hardware_interface
  pluginlib
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
  add_dependencies(dummy_app ${catkin_EXPORTED_TARGETS})
  target_link_libraries(dummy_app ${PROJECT_NAME} ${catkin_LIBRARIES})

  add_executable(cm_test test/cm_test.cpp)
  add_dependencies(cm_test ${catkin_EXPORTED_TARGETS})
  target_link_libraries(cm_test ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

  catkin_add_nosetests(test)

  add_rostest(test/cm_test.test DEPENDENCIES dummy_app cm_test)

  add_rostest(test/cm_msgs_utils_rostest.test)

  add_rostest(test/controller_manager_scripts.test DEPENDENCIES dummy_app)

  add_rostest(test/controller_manager_interface_test.test DEPENDENCIES dummy_app)
endif()


#############
## Install ##
#############

# NOTE: Libraries and plugins required for tests are installed since CI
# runs tests out of the install space rather than the devel space

if(CATKIN_ENABLE_TESTING)

  # Install targets
  install(TARGETS ${PROJECT_NAME} 
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

  install(TARGETS dummy_app cm_test 
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  # Install plugins
  install(FILES test_controllers_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

  catkin_install_python(PROGRAMS test/controller_manager_scripts.py
	                         test/controller_manager_interface_test.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
