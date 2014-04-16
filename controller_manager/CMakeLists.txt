cmake_minimum_required(VERSION 2.8.3)
project(controller_manager)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS controller_interface controller_manager_msgs hardware_interface realtime_tools pluginlib)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

# Declare catkin package
catkin_package(
  DEPENDS boost
  CATKIN_DEPENDS controller_interface controller_manager_msgs hardware_interface realtime_tools pluginlib
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  )

add_library(${PROJECT_NAME}
  src/controller_manager.cpp
  include/controller_manager/controller_manager.h
  include/controller_manager/controller_loader_interface.h
  include/controller_manager/controller_loader.h
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

if(catkin_EXPORTED_TARGETS)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
endif()

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(PROGRAMS scripts/spawner scripts/unspawner scripts/controller_manager
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_python_setup()
