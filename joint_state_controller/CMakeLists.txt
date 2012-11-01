cmake_minimum_required(VERSION 2.8.3)

include(../catkin_basic.cmake NO_POLICY_SCOPE)
catkin_basic(INCLUDE include LIBRARIES joint_state_controller)

add_library(joint_state_controller SHARED src/joint_state_controller.cpp)
target_link_libraries(joint_state_controller ${Boost_LIBRARIES} ${catkin_LIBRARIES})
