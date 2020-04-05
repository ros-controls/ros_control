cmake_minimum_required(VERSION 3.0.2)
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
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_parser
    ${PROJECT_NAME}_loader
    ${PROJECT_NAME}_loader_plugins
  CATKIN_DEPENDS
    hardware_interface
    pluginlib
    roscpp
  DEPENDS
    TinyXML
)


###########
## Build ##
###########

# Specify header include paths
include_directories(include ${catkin_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS})

# Transmission parser library
add_library(${PROJECT_NAME}_parser
  include/transmission_interface/transmission_parser.h
  src/transmission_parser.cpp
)
target_link_libraries(${PROJECT_NAME}_parser ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})

# Transmission loader library
add_library(${PROJECT_NAME}_loader
  include/transmission_interface/transmission_interface_loader.h
  include/transmission_interface/transmission_loader.h
  src/transmission_interface_loader.cpp
  src/transmission_loader.cpp
)
target_link_libraries(${PROJECT_NAME}_loader
  ${PROJECT_NAME}_parser
  ${catkin_LIBRARIES}
  ${TinyXML_LIBRARIES}
)

add_library(${PROJECT_NAME}_loader_plugins
  include/transmission_interface/bidirectional_effort_joint_interface_provider.h
  include/transmission_interface/bidirectional_position_joint_interface_provider.h
  include/transmission_interface/bidirectional_velocity_joint_interface_provider.h
  include/transmission_interface/differential_transmission_loader.h
  include/transmission_interface/effort_joint_interface_provider.h
  include/transmission_interface/four_bar_linkage_transmission_loader.h
  include/transmission_interface/joint_state_interface_provider.h
  include/transmission_interface/position_joint_interface_provider.h
  include/transmission_interface/simple_transmission_loader.h
  include/transmission_interface/velocity_joint_interface_provider.h
  src/bidirectional_effort_joint_interface_provider.cpp
  src/bidirectional_position_joint_interface_provider.cpp
  src/bidirectional_velocity_joint_interface_provider.cpp
  src/differential_transmission_loader.cpp
  src/effort_joint_interface_provider.cpp
  src/four_bar_linkage_transmission_loader.cpp
  src/joint_state_interface_provider.cpp
  src/position_joint_interface_provider.cpp
  src/simple_transmission_loader.cpp
  src/velocity_joint_interface_provider.cpp
)
target_link_libraries(${PROJECT_NAME}_loader_plugins ${PROJECT_NAME}_loader)


#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(resource_retriever REQUIRED)
  include_directories(${resource_retriever_INCLUDE_DIRS})

  catkin_add_gtest(simple_transmission_test                       test/simple_transmission_test.cpp)

  catkin_add_gtest(differential_transmission_test                 test/differential_transmission_test.cpp)

  catkin_add_gtest(four_bar_linkage_transmission_test             test/four_bar_linkage_transmission_test.cpp)

  catkin_add_gtest(transmission_interface_test                    test/transmission_interface_test.cpp)
  target_link_libraries(transmission_interface_test               ${catkin_LIBRARIES})

  catkin_add_gtest(transmission_parser_test                       test/transmission_parser_test.cpp)
  target_link_libraries(transmission_parser_test                  ${PROJECT_NAME}_parser ${resource_retriever_LIBRARIES})

  catkin_add_gtest(simple_transmission_loader_test                test/simple_transmission_loader_test.cpp)
  target_link_libraries(simple_transmission_loader_test           ${PROJECT_NAME}_parser ${resource_retriever_LIBRARIES})

  catkin_add_gtest(differential_transmission_loader_test          test/differential_transmission_loader_test.cpp)
  target_link_libraries(differential_transmission_loader_test     ${PROJECT_NAME}_parser ${resource_retriever_LIBRARIES})

  catkin_add_gtest(four_bar_linkage_transmission_loader_test      test/four_bar_linkage_transmission_loader_test.cpp)
  target_link_libraries(four_bar_linkage_transmission_loader_test ${PROJECT_NAME}_parser ${resource_retriever_LIBRARIES})

  catkin_add_gtest(transmission_interface_loader_test             test/transmission_interface_loader_test.cpp)
  target_link_libraries(transmission_interface_loader_test        transmission_interface_loader
                                                                  ${resource_retriever_LIBRARIES})
endif()


#############
## Install ##
#############

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install targets
install(TARGETS ${PROJECT_NAME}_loader
                ${PROJECT_NAME}_loader_plugins
                ${PROJECT_NAME}_parser
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install plugins
install(FILES ros_control_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
