cmake_minimum_required(VERSION 2.8.3)
project(extended_robot_hw_tests)

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    pluginlib
    transmission_interface)

find_package(TinyXML REQUIRED)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}_loader_plugins
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    pluginlib
  DEPENDS
    TinyXML
)


###########
## Build ##
###########

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_loader_plugins
  src/extended_simple_transmission_loader.cpp
  src/foo_joint_interface_provider.cpp)
#  src/bar_joint_interface_provider.cpp)

target_link_libraries(${PROJECT_NAME}_loader_plugins ${catkin_LIBRARIES})



install(FILES extended_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

  find_package(catkin REQUIRED
    COMPONENTS
      hardware_interface
      resource_retriever
      transmission_interface
      urdf
  )

  find_package(rostest REQUIRED)

  include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

  catkin_add_gtest(extended_robot_hw_test  test/extended_robot_hw_test.cpp)

  target_link_libraries(extended_robot_hw_test
      ${catkin_LIBRARIES})

endif()

