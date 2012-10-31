cmake_minimum_required(VERSION 2.8.3)

include(/home/wim/stacks/groovy/catkin_easy/catkin_easy.cmake NO_POLICY_SCOPE)
catkin_basic(INCLUDE include)

add_library(${PROJECT_NAME} SHARED src/dummy_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
