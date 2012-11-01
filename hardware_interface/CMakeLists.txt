cmake_minimum_required(VERSION 2.8.3)

include(../catkin_basic.cmake NO_POLICY_SCOPE)
catkin_basic(INCLUDE include LIBRARIES dummy_hardware_interface)

add_library(dummy_hardware_interface SHARED src/dummy_interface.cpp)
target_link_libraries(dummy_hardware_interface ${Boost_LIBRARIES} ${catkin_LIBRARIES})

