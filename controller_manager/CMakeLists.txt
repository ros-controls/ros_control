# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(controller_manager)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS controller_interface controller_manager_msgs hardware_interface realtime_tools pluginlib)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/controller_manager.cpp
  include/controller_manager/controller_manager.h
  include/controller_manager/controller_loader_interface.h
  include/controller_manager/controller_loader.h)

# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS controller_interface controller_manager_msgs hardware_interface realtime_tools pluginlib
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)
