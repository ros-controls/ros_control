cmake_minimum_required(VERSION 2.8.3)

include(/home/wim/stacks/groovy/catkin_easy/catkin_easy.cmake NO_POLICY_SCOPE)
catkin_basic(INCLUDE include LIBRARIES controller_manager)

add_library(controller_manager SHARED src/controller_manager.cpp)
target_link_libraries(controller_manager ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(dummy src/dummy.cpp)
target_link_libraries(dummy controller_manager ${Boost_LIBRARIES} ${catkin_LIBRARIES})