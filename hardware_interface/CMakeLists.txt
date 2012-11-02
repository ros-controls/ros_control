cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_basic REQUIRED)
catkin_basic(INCLUDE include LIBRARIES dummy_hardware_interface)

add_library(dummy_hardware_interface SHARED src/dummy_interface.cpp)
target_link_libraries(dummy_hardware_interface ${Boost_LIBRARIES} ${catkin_LIBRARIES})

