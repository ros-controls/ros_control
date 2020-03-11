cmake_minimum_required(VERSION 3.0.2)
project(controller_manager_msgs)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

catkin_python_setup()

# Add message files
add_message_files(FILES
  ControllerState.msg
  ControllerStatistics.msg
  ControllersStatistics.msg
  HardwareInterfaceResources.msg
)

# Add service files
add_service_files(FILES
  ListControllerTypes.srv
  ListControllers.srv
  LoadController.srv
  ReloadControllerLibraries.srv
  SwitchController.srv
  UnloadController.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)
