cmake_minimum_required(VERSION 3.0.2)
project(joint_limits_interface)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  roscpp
  urdf
)

# Declare a catkin package
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    hardware_interface
    roscpp
    urdf
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
  find_package(rostest REQUIRED)

  catkin_add_gtest(joint_limits_interface_test test/joint_limits_interface_test.cpp)
  target_link_libraries(joint_limits_interface_test ${catkin_LIBRARIES})

  catkin_add_gtest(joint_limits_urdf_test test/joint_limits_urdf_test.cpp)
  target_link_libraries(joint_limits_urdf_test ${catkin_LIBRARIES})

  add_rostest_gtest(joint_limits_rosparam_test
    test/joint_limits_rosparam.test
    test/joint_limits_rosparam_test.cpp
  )
  target_link_libraries(joint_limits_rosparam_test ${catkin_LIBRARIES})
endif()


#############
## Install ##
#############

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
