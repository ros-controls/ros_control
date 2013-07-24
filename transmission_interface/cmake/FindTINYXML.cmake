################################################################################
#
# CMake script for finding TinyXML.
# If the optional TINYXML_ROOT_DIR environment or cmake variable exists,
# header files and libraries will be searched in the ${TINYXML_ROOT_DIR}/include
# and ${TINYXML_ROOT_DIR}/libs directories, respectively.
# Otherwise the default CMake search process will be used.
#
# This script creates the following variables:
#  TINYXML_FOUND: Boolean that indicates if the package was found
#  TINYXML_INCLUDE_DIRS: Paths to the necessary header files
#  TINYXML_LIBRARIES: Package libraries
#
# This is a typical way to use this module:
#
#  find_package(TINYXML REQUIRED)
#  ...
#  include_directories(${TINYXML_INCLUDE_DIRS} ...)
#  ...
#  target_link_libraries(my_target ${TINYXML_LIBRARIES})
#
################################################################################

# Get hint from environment variable (if any)
if(NOT TINYXML_ROOT_DIR AND DEFINED ENV{TINYXML_ROOT_DIR})
  set(TINYXML_ROOT_DIR "$ENV{TINYXML_ROOT_DIR}" CACHE PATH
      "TinyXML base directory location (optional, used for nonstandard installation paths)")
endif()

# Search path for nonstandard locations
if(TINYXML_ROOT_DIR)
  set(TINYXML_INCLUDE_PATH PATHS "${TINYXML_ROOT_DIR}/include" NO_DEFAULT_PATH)
  set(TINYXML_LIBRARY_PATH PATHS "${TINYXML_ROOT_DIR}/lib"     NO_DEFAULT_PATH)
endif()

# Find headers and libraries
find_path(TINYXML_INCLUDE_DIR NAMES tinyxml.h PATH_SUFFIXES "tinyxml" ${TINYXML_INCLUDE_PATH})
find_library(TINYXML_LIBRARY  NAMES tinyxml   PATH_SUFFIXES "tinyxml" ${TINYXML_LIBRARY_PATH})

mark_as_advanced(TINYXML_INCLUDE_DIR
                 TINYXML_LIBRARY)

# Output variables generation
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TinyXML DEFAULT_MSG TINYXML_LIBRARY
                                                      TINYXML_INCLUDE_DIR)

if(TINYXML_FOUND)
  set(TINYXML_INCLUDE_DIRS ${TINYXML_INCLUDE_DIR})
  set(TINYXML_LIBRARIES ${TINYXML_LIBRARY})
endif()
