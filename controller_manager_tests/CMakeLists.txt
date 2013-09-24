cmake_minimum_required(VERSION 2.8.3)
project(controller_manager_tests)

if(USE_ROSBUILD)

  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

  rosbuild_init()

  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

  rosbuild_add_library(${PROJECT_NAME} 
    src/my_robot_hw.cpp
    include/controller_manager_tests/my_robot_hw.h
    src/effort_test_controller.cpp
    include/controller_manager_tests/effort_test_controller.h
    src/my_dummy_controller.cpp
    include/controller_manager_tests/my_dummy_controller.h)

  rosbuild_add_executable(dummy_app src/dummy_app.cpp)
  target_link_libraries(dummy_app ${PROJECT_NAME})

  rosbuild_add_executable(cm_test test/cm_test.cpp)
  rosbuild_add_gtest_build_flags(cm_test)

  rosbuild_add_rostest(test/cm_test.test)

else()

  # Load catkin and all dependencies required for this package
  find_package(catkin REQUIRED COMPONENTS rostest controller_manager controller_interface control_toolbox)

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
    src/my_dummy_controller.cpp
    include/controller_manager_tests/my_dummy_controller.h
    )

  add_executable(dummy_app src/dummy_app.cpp)
  target_link_libraries(dummy_app ${PROJECT_NAME} ${catkin_LIBRARIES})

  if(CATKIN_ENABLE_TESTING)
    add_executable(cm_test test/cm_test.cpp)
    add_dependencies(tests cm_test)
    target_link_libraries(cm_test ${GTEST_LIBRARIES} ${catkin_LIBRARIES})
    add_rostest(test/cm_test.test)
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

endif()
