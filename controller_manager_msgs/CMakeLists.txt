cmake_minimum_required(VERSION 2.8.3)
project(controller_manager_msgs)

find_package(catkin REQUIRED COMPONENTS genmsg)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package()

add_service_files(DIRECTORY srv FILES ListControllers.srv)
generate_messages()


# install the include folder
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
