cmake_minimum_required(VERSION 2.8.3)
project(combined_robot_hw_tests)

find_package(catkin REQUIRED COMPONENTS
  combined_robot_hw
  controller_manager
  controller_manager_tests
  hardware_interface
  roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS combined_robot_hw hardware_interface roscpp
)

add_library(${PROJECT_NAME}
  src/my_robot_hw_1.cpp
  src/my_robot_hw_2.cpp
  src/my_robot_hw_3.cpp
  src/my_robot_hw_4.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(combined_robot_hw_dummy_app src/dummy_app.cpp)
target_link_libraries(combined_robot_hw_dummy_app ${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)
  add_rostest_gtest(combined_robot_hw_test
    test/combined_robot_hw_test.test
    test/combined_robot_hw_test.cpp
  )
  target_link_libraries(combined_robot_hw_test ${PROJECT_NAME} ${catkin_LIBRARIES})

  add_rostest_gtest(combined_robot_hw_cm_test
    test/cm_test.test
    test/cm_test.cpp
  )
  target_link_libraries(combined_robot_hw_cm_test ${PROJECT_NAME} ${catkin_LIBRARIES})

endif()

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES test_robot_hw_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

