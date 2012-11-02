cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_basic REQUIRED)
catkin_basic(INCLUDE include LIBRARIES joint_state_controller)

add_library(joint_state_controller SHARED src/joint_state_controller.cpp)
target_link_libraries(joint_state_controller ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(dummy_controller src/dummy.cpp)
target_link_libraries(dummy_controller joint_state_controller ${Boost_LIBRARIES} ${catkin_LIBRARIES})