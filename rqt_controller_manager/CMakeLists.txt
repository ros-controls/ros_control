cmake_minimum_required(VERSION 2.8.3)
project(rqt_controller_manager)

find_package(catkin REQUIRED COMPONENTS)
catkin_python_setup()
catkin_package()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/rqt_controller_manager
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
