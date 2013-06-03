# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(controller_manager_tests)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS controller_manager controller_interface control_toolbox)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

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

# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS controller_manager controller_interface control_toolbox
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

catkin_add_gtest(cm_test test/cm_test.cpp)
