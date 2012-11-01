cmake_minimum_required(VERSION 2.8.3)


# select the catkin dependencies from deps
macro(select_catkin_dependencies PREFIX DEPS)
  set(${PREFIX}_DEPENDENCIES "")
  set(${PREFIX}_MESSAGE_DEPENDENCIES "")
  foreach(pkg ${DEPS})
    find_package(${pkg})
    if (${${pkg}_FOUND_CATKIN_PROJECT})
      list(APPEND ${PREFIX}_DEPENDENCIES ${pkg})
    endif()
    foreach(workspace ${CATKIN_WORKSPACES})
      if (EXISTS ${workspace}/share/${pkg}/cmake/${pkg}-msg-paths.cmake)
	list(APPEND ${PREFIX}_MESSAGE_DEPENDENCIES ${pkg})	
      endif()
    endforeach()
  endforeach()
  
  message(${_CATKIN_CURRENT_PACKAGE} " has " ${PREFIX} " message dependencies:")
  foreach(pkg ${${PREFIX}_MESSAGE_DEPENDENCIES})
    message("  -"${pkg})
  endforeach()
  message(${_CATKIN_CURRENT_PACKAGE} " has " ${PREFIX} " dependencies:")
  foreach(pkg ${${PREFIX}_DEPENDENCIES})
    message("  -"${pkg})
  endforeach()
endmacro()




macro(catkin_basic)
  # arguments
  cmake_parse_arguments(ARG "" "INCLUDE" "" ${ARGN})
  cmake_parse_arguments(ARG "" "LIBRARIES" "" ${ARGN})

#  message(${ARG_LIBRARIES})

  # parse package.xml
  _catkin_package_xml(${CMAKE_CURRENT_BINARY_DIR}/catkin_generated)

  # define our project
  project(${_CATKIN_CURRENT_PACKAGE})

  # call find_package on build dependencies
  if (DEFINED ${_CATKIN_CURRENT_PACKAGE}_BUILD_DEPENDS)  
    select_catkin_dependencies(BUILD "${${_CATKIN_CURRENT_PACKAGE}_BUILD_DEPENDS}")
    find_package(catkin REQUIRED COMPONENTS ${BUILD_DEPENDENCIES})
  endif()

  # generate messages
  file(GLOB MESSAGES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg" msg/*.msg)
  list(LENGTH MESSAGES NUM_MESSAGES)
  if (${NUM_MESSAGES} GREATER 0)
    add_message_files(DIRECTORY msg FILES ${MESSAGES})
  endif()

  # generate services
  file(GLOB SERVICES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/srv" srv/*.srv)
  list(LENGTH SERVICES NUM_SERVICES)
  if (${NUM_SERVICES} GREATER 0)
    add_service_files(DIRECTORY srv FILES ${SERVICES})
  endif()

  # run generation
  if (${NUM_SERVICES} GREATER 0 OR ${NUM_MESSAGES} GREATER 0)
    message(${BUILD_DEPENDENCIES})
    generate_messages(DEPENDENCIES ${BUILD_MESSAGE_DEPENDENCIES})
  endif()

  # call catkin_package on run dependencies
  if (DEFINED ${_CATKIN_CURRENT_PACKAGE}_RUN_DEPENDS)
    select_catkin_dependencies(RUN "${${_CATKIN_CURRENT_PACKAGE}_RUN_DEPENDS}")
    message("  Export my include dir: "${ARG_INCLUDE})
    message("  Export my lib dir: "${ARG_LIBRARIES})
    catkin_package(DEPENDS ${RUN_DEPENDENCIES}
      INCLUDE_DIRS ${ARG_INCLUDE}
      LIBRARIES ${ARG_LIBRARIES})
  else()
    message("  Export my include dir: "${ARG_INCLUDE})
    message("  Export my lib dir: "${ARG_LIBRARIES})
    catkin_package(
      INCLUDE_DIRS ${ARG_INCLUDE}
      LIBRARIES ${ARG_LIBRARIES})
  endif()


  # include ourselves and all catkin stuff
  include_directories(${catkin_INCLUDE_DIRS})
  include_directories(${ARG_INCLUDE})

  # install target
  #install(DIRECTORY include/
  #        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  #)
endmacro()
