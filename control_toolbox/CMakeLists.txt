cmake_minimum_required(VERSION 2.8.3)
project(control_toolbox)

if(USE_ROSBUILD)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()
  #rosbuild_genmsg()
  rosbuild_gensrv()
  rosbuild_add_library(control_toolbox
    src/pid.cpp
    src/pid_gains_setter.cpp
    src/sine_sweep.cpp
    src/dither.cpp
    src/sinusoid.cpp
    src/limited_proxy.cpp
    )
  target_link_libraries(control_toolbox tinyxml)

  # rosbuild_add_executable(test_linear test/linear.cpp)

  # Tests
  rosbuild_add_gtest(test/pid_tests test/pid_tests.cpp) 
  target_link_libraries(test/pid_tests control_toolbox)

else()

  # Load catkin and all dependencies required for this package
  find_package(catkin REQUIRED COMPONENTS rosconsole message_generation tf roscpp angles)

  include_directories(
    include 
    ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

  # Add services and generate them
  add_service_files(
    FILES
    SetPidGains.srv
    )

  generate_messages(
    DEPENDENCIES std_msgs
    )

  add_library(${PROJECT_NAME}
    src/pid.cpp
    src/pid_gains_setter.cpp
    src/sine_sweep.cpp
    src/dither.cpp
    src/sinusoid.cpp
    src/limited_proxy.cpp
    )

  target_link_libraries(control_toolbox tinyxml)
  add_dependencies(control_toolbox ${PROJECT_NAME}_gencpp)

  # Declare catkin package
  catkin_package(
    DEPENDS tinyxml
    CATKIN_DEPENDS rosconsole tf roscpp angles 
    INCLUDE_DIRS include 
    LIBRARIES ${PROJECT_NAME} 
    )

  # Tests
  catkin_add_gtest(pid_tests test/pid_tests.cpp) 
  target_link_libraries(pid_tests ${PROJECT_NAME})

  # add_executable(test_linear test/linear.cpp)

  # Install
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

  install(DIRECTORY scripts/
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

endif()
