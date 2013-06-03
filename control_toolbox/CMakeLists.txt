# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(control_toolbox)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS rosconsole message_generation tf roscpp angles)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
include_directories(include/control_toolbox/eigen2)

add_service_files(
  FILES
  SetPidGains.srv
)

add_library(control_toolbox
 src/pid.cpp
 src/pid_gains_setter.cpp
 src/sine_sweep.cpp
 src/dither.cpp
 src/sinusoid.cpp
 src/limited_proxy.cpp
 )

target_link_libraries(control_toolbox tinyxml)
add_dependencies(control_toolbox control_toolbox_gencpp)
 

generate_messages(
  DEPENDENCIES std_msgs
)

# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS rosconsole tf roscpp angles tinyxml
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include
    LIBRARIES control_toolbox
)

# Tests
catkin_add_gtest(pid_tests test/pid_tests.cpp) 
target_link_libraries(pid_tests control_toolbox)

# add_executable(test_linear test/linear.cpp)
