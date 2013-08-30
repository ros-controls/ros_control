cmake_minimum_required(VERSION 2.8.3)
project(transmission_interface)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED
  hardware_interface
  cmake_modules
  resource_retriever
)

# Include a custom cmake file for TinyXML
find_package(TinyXML REQUIRED)

# Declare a catkin package
catkin_package(
  LIBRARIES
    ${PROJECT_NAME}_parser
  INCLUDE_DIRS
    include
  DEPENDS
    TinyXML
    resource_retriever
)

###########
## Build ##
###########

# Build
include_directories(include ${catkin_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS})

# Transmission Paraser Library
add_library(${PROJECT_NAME}_parser
  src/transmission_parser.cpp
)
target_link_libraries(${PROJECT_NAME}_parser ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})

#############
## Install ##
#############

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_parser
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )

#############
## Testing ##
#############

<<<<<<< HEAD
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(simple_transmission_test           test/simple_transmission_test.cpp)
  catkin_add_gtest(differential_transmission_test     test/differential_transmission_test.cpp)
  catkin_add_gtest(four_bar_linkage_transmission_test test/four_bar_linkage_transmission_test.cpp)
  catkin_add_gtest(transmission_interface_test        test/transmission_interface_test.cpp)
  target_link_libraries(transmission_interface_test ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})
endif()

catkin_add_gtest(transmission_parser_test           test/transmission_parser_test.cpp)
target_link_libraries(transmission_parser_test   ${PROJECT_NAME}_parser)
