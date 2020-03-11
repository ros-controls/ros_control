cmake_minimum_required(VERSION 3.0.2)
project(controller_manager)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager_msgs
  hardware_interface
  pluginlib
  roscpp
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
    controller_manager_msgs
    hardware_interface
    pluginlib
    roscpp
)


###########
## Build ##
###########

# Specify header include paths
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  include/controller_manager/controller_loader.h
  include/controller_manager/controller_loader_interface.h
  include/controller_manager/controller_manager.h
  src/controller_manager.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(${PROJECT_NAME}_hwi_switch_test
    test/hwi_switch_test.test
    test/hwi_switch_test.cpp
  )
  add_dependencies(${PROJECT_NAME}_hwi_switch_test ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}_hwi_switch_test ${PROJECT_NAME} ${catkin_LIBRARIES})

  add_rostest_gmock(${PROJECT_NAME}_hwi_update_test
    test/hwi_update_test.test
    test/hwi_update_test.cpp
  )
  add_dependencies(${PROJECT_NAME}_hwi_update_test ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}_hwi_update_test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()


#############
## Install ##
#############

# Install python scripts
catkin_install_python(PROGRAMS scripts/controller_group
                               scripts/controller_manager
                               scripts/spawner
                               scripts/unspawner
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install targets
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
