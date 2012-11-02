cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_basic REQUIRED)
catkin_basic(INCLUDE include LIBRARIES controller_manager)

add_library(controller_manager SHARED src/controller_manager.cpp)
target_link_libraries(controller_manager ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(dummy_cm src/dummy.cpp)
target_link_libraries(dummy_cm controller_manager ${Boost_LIBRARIES} ${catkin_LIBRARIES})