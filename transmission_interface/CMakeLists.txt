cmake_minimum_required(VERSION 2.8.3)
project(transmission_interface)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  hardware_interface
  pluginlib
  roscpp
)

# Include a custom cmake file for TinyXML
find_package(TinyXML REQUIRED)

# Declare a catkin package
catkin_package(
  LIBRARIES
    ${PROJECT_NAME}_parser
    ${PROJECT_NAME}_loader
    ${PROJECT_NAME}_loader_plugins
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    pluginlib
    roscpp
  DEPENDS
    TinyXML
)

###########
## Build ##
###########

# Build
include_directories(include ${catkin_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS})

# Transmission parser Library
add_library(${PROJECT_NAME}_parser
  src/transmission_parser.cpp include/transmission_interface/transmission_parser.h
)
target_link_libraries(${PROJECT_NAME}_parser ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})


# Transmission loader library
add_library(${PROJECT_NAME}_loader
  src/transmission_loader.cpp           include/transmission_interface/transmission_loader.h
  src/transmission_interface_loader.cpp include/transmission_interface/transmission_interface_loader.h
)
target_link_libraries(${PROJECT_NAME}_loader
  ${PROJECT_NAME}_parser
  ${catkin_LIBRARIES}
  ${TinyXML_LIBRARIES})


add_library(${PROJECT_NAME}_loader_plugins
  src/simple_transmission_loader.cpp                      include/transmission_interface/simple_transmission_loader.h
  src/differential_transmission_loader.cpp                include/transmission_interface/differential_transmission_loader.h
  src/four_bar_linkage_transmission_loader.cpp            include/transmission_interface/four_bar_linkage_transmission_loader.h
  src/joint_state_interface_provider.cpp                  include/transmission_interface/joint_state_interface_provider.h
  src/position_joint_interface_provider.cpp               include/transmission_interface/position_joint_interface_provider.h
  src/velocity_joint_interface_provider.cpp               include/transmission_interface/velocity_joint_interface_provider.h
  src/effort_joint_interface_provider.cpp                 include/transmission_interface/effort_joint_interface_provider.h
  src/bidirectional_position_joint_interface_provider.cpp include/transmission_interface/bidirectional_position_joint_interface_provider.h
  src/bidirectional_velocity_joint_interface_provider.cpp include/transmission_interface/bidirectional_velocity_joint_interface_provider.h
  src/bidirectional_effort_joint_interface_provider.cpp   include/transmission_interface/bidirectional_effort_joint_interface_provider.h)
target_link_libraries(${PROJECT_NAME}_loader_plugins ${PROJECT_NAME}_loader)


#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_parser
                ${PROJECT_NAME}_loader
                ${PROJECT_NAME}_loader_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES ros_control_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

  find_package(Boost REQUIRED COMPONENTS system thread)
  find_package(catkin REQUIRED COMPONENTS resource_retriever roscpp)

  catkin_add_gtest(simple_transmission_test test/simple_transmission_test.cpp)
  target_link_libraries(simple_transmission_test ${Boost_LIBRARIES})

  catkin_add_gtest(differential_transmission_test test/differential_transmission_test.cpp)
  target_link_libraries(differential_transmission_test ${Boost_LIBRARIES})

  catkin_add_gtest(four_bar_linkage_transmission_test test/four_bar_linkage_transmission_test.cpp)
  target_link_libraries(four_bar_linkage_transmission_test ${Boost_LIBRARIES})

  catkin_add_gtest(transmission_interface_test test/transmission_interface_test.cpp)
  target_link_libraries(transmission_interface_test ${TinyXML_LIBRARIES} ${catkin_LIBRARIES})

  catkin_add_gtest(transmission_parser_test test/transmission_parser_test.cpp)
  target_link_libraries(transmission_parser_test ${PROJECT_NAME}_parser ${catkin_LIBRARIES})

  catkin_add_gtest(simple_transmission_loader_test test/simple_transmission_loader_test.cpp)
  target_link_libraries(simple_transmission_loader_test ${PROJECT_NAME}_parser ${catkin_LIBRARIES})

  catkin_add_gtest(differential_transmission_loader_test test/differential_transmission_loader_test.cpp)
  target_link_libraries(differential_transmission_loader_test ${PROJECT_NAME}_parser ${catkin_LIBRARIES})

  catkin_add_gtest(four_bar_linkage_transmission_loader_test test/four_bar_linkage_transmission_loader_test.cpp)
  target_link_libraries(four_bar_linkage_transmission_loader_test ${PROJECT_NAME}_parser ${catkin_LIBRARIES})

  catkin_add_gtest(transmission_interface_loader_test test/transmission_interface_loader_test.cpp)
  target_link_libraries(transmission_interface_loader_test ${PROJECT_NAME}_parser
                                                          transmission_interface_loader
                                                          ${catkin_LIBRARIES})
endif()
