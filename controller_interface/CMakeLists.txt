cmake_minimum_required(VERSION 3.0.2)
project(controller_interface)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  roscpp
)

# Declare a catkin package
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    hardware_interface
    roscpp
)


###########
## Build ##
###########

# Specify header include paths
include_directories(include ${catkin_INCLUDE_DIRS})


#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  catkin_add_gmock(controller_base_test test/controller_base_test.cpp)
  target_link_libraries(controller_base_test ${catkin_LIBRARIES})
endif()


#############
## Install ##
#############

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
