^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package transmission_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.13.3 (2018-04-26)
-------------------
* Introduce shared_ptr typedefs
* Update maintainers
* Fix catkin_lint errors and warnings
* fix license string
* Update transmission parser to parse the joint role
* Contributors: Bence Magyar, Patrick Holthaus, jlack1987

0.13.2 (2018-04-16)
-------------------
* migrate to new class list macros header
* migrate classloader headers
* Contributors: Mathias Lüdtke

0.13.1 (2018-03-26)
-------------------

0.13.0 (2017-12-23)
-------------------

0.12.0 (2017-08-05)
-------------------
* Add unit tests for new bidirectional joint interface providers
* Add bidirectional joint interface providers
* Add inverse transmission interfaces to TransmissionLoaderData
* Contributors: Jordan Lack

0.11.5 (2017-06-28)
-------------------

0.11.4 (2017-02-14)
-------------------

0.11.3 (2016-12-07)
-------------------

0.11.2 (2016-11-28)
-------------------
* Add Enrique and Bence to maintainer list
* Clean up export leftovers from rosbuild
* Convert to format2, fix dependency in cmake
* Contributors: Bence Magyar

0.11.1 (2016-08-18)
-------------------

0.11.0 (2016-05-23)
-------------------

0.10.1 (2016-04-23)
-------------------
* Remove control_toolbox dependency. Fix thread linking error coming from removal of dependency.
* Contributors: Bence Magyar

0.10.0 (2015-11-20)
-------------------
* Allow loading transmissions from a vector of TransmissionInfo instances.
* Address -Wunused-parameter warnings
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.3 (2015-05-05)
------------------

0.9.2 (2015-05-04)
------------------

0.9.1 (2014-11-03)
------------------

0.9.0 (2014-10-31)
------------------
* Buildsystem and documentation fixes
* Contributors: Adolfo Rodriguez Tsouroukdissian, shadowmanos

0.8.2 (2014-06-25)
------------------

0.8.1 (2014-06-24)
------------------

0.8.0 (2014-05-12)
------------------
* Add developer documentation.
* Build script fixes.
  - Add missing libraries to catkin_package call.
  - Gate tests with CATKIN_ENABLE_TESTING.
  - Add missing files to install target.
* Fix possible memory corruption in tests.
* Perform sanity checks on members, not parameters.
  - The result is the same, but this is more uniform with the rest of the code.
* Enable joint reduction spec for 4-bar linkages.
  - As in the differential transmission, it's convenient to specify an additional
  mechanical reduction on the joint output. This is especially convenient for
  flipping the rotation direction of a joint (negative reduction value).
  - Update URDF loader.
  - Update documentation and tests.
* Trivial, cosmetic fixes.
* C++11 compatibility fixes.
* Fix resource check for multi-dof transmisisons.
* Efficiency fix.
  - cppcheck flagged a [passedByValue] warning. Using const references instead.
* Fix compiler warning.
* Fix license header in some files.
* Test transmission handle duplication.
* Use less pointers in transmission loader data.
  - Only RobotHW and RobotTransmission instances are pointers as they are owned
  by the robot hardware abstraction. The rest are plain members whose lifetime
  is bound to the loader struct.
* Trivial test addition.
* Remove unnecessary header dependencies.
* Catkin fixes.
* Fix bug when adding multiple transmissions.
  - std::vectors were being used to store raw joint data, and when new transmissions
  were added, push_back()s would (potentially) reallocate the vectors and
  invalidate already stored pointers in hardware_interfaces. We now use std::map.
  - Move plugin implementations to a separate library.
  - Export link libraries to the outside.
  - More complete tests.
* Log message change.
* Test greceful error-out with unsupported features.
* Add four-bar-linkage transmission parser.
* Add differential drive transmission parser.
* Move common XML parsing code to TransmissionLoader
  Mechanical reductions, offsets and roles are used by many transmission types.
  The TransmissionLoader base class exposes convenience methods for parsing these
  elements.
* Remove dead code.
* Update loader test, better log statements.
* First draft of transmission loading.
  - Only simple transmission type currently supported.
  - Can load forward map for act->jnt state and jnt->act pos,vel.eff commands.
  - Partial testing.
* Add class for holding transmission interfaces.
  - Mirrors hardware_interface::RobotHW, but for transmissions.
* Allow multiple hw interfaces, Fix `#112 <https://github.com/ros-controls/ros_control/issues/112>`_, and test.
  - Allow to specify multiple hardware interfaces for joints and actuators.
  - Fix invalid xml_element tag. Contents are now stored as a string.
  - Unit test parser.
* Remove rosbuild artifacts. Fix `#154 <https://github.com/ros-controls/ros_control/issues/154>`_.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.7.2 (2014-04-01)
------------------

0.7.1 (2014-03-31)
------------------

0.7.0 (2014-03-28)
------------------

0.6.0 (2014-02-05)
------------------
* Add accessors to get transmission configuration.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.5.8 (2013-10-11)
------------------
* Renamed manifest.xml to prevent conflicts with rosdep
* CMakeLists fix to fit with OpenEmbedded/Yocto meta-ros layer.
  Increase the compatibility of the ros_control code with
  meta-ros, an OpenEmbedded/Yocto layer that provides recipes for ROS
  packages disabling catking checking the variable CATKIN_ENABLE_TESTING.
* Fix license header in some files.
* Fix cppcheck uninit'd variable warnings in tests.

0.5.7 (2013-07-30)
------------------
* Fix for building ros_control
* Updated CHANGELOG

0.5.6 (2013-07-29)
------------------

* NOTE: ros_control now requires ros-*-cmake-modules for source-installations. Install via rosdep or manually
* Removed the local FindTINYXML.cmake and switched to catkin's cmake_modules version
* Installed missing transmission_interface_library

0.5.5 (2013-07-23)
------------------
* transmission_interface: fixup finding tinyxml

0.5.4 (2013-07-23)
------------------

0.5.3 (2013-07-22)
------------------
* Duplicated URDF's method of including tinyxml

0.5.2 (2013-07-22)
------------------
* Trivial cleanup
* tinyxml include dir fix

0.5.1 (2013-07-19)
------------------
* Added new maintainer
* Attempt to fix transmission interface tinyxml build error

0.5.0 (2013-07-16)
------------------
* Minor Doxygen fixes.
  - Revert back to using \file instead of \brief, as the latter was documenting
  the namespace and not the file scope.
  - Escape angular brackets on XML tag documentation, as Doxygen was parsing them
  printing warnings.
  @davetcoleman
* Code consistency fixes.
  - Add missing header guard.
  - Make existing header guards comply with the NAMESPACE_CLASS_H convention.
  - Make Doxygen structural commands start with '\' instead of '@', as most of the
  new ros_control code.
  - Remove trailing whitespaces.
  - Remove commented-out code used for debugging.
* Build script fixes.
  - Add missing tinyxml dependency.
  - Drop unnecessary Boost dependency.
  - Add URDF parsing code to rosbuild.
* Add meta tags to packages not specifying them.
  - Website, bugtracker, repository.
* Documentation improvements.
  - More consistency between transmission and joint limits interfaces doc.
  - Make explicit that these interfaces are not meant to be used by controllers,
  but by the robot abstraction.
* Transmission parsing
* Merged hydro-devel into master
* Fix doc typo. Refs `#78 <https://github.com/davetcoleman/ros_control/issues/78>`_.
* Tests build.
* Reneamed Github repo in documentation to ros-controls
* Make specific transmission interfaces proper types.
  - Proper types instead of namespaces allow to provide less cryptic feedback.
  * Using typedefs:
  "transmission_interface::TransmissionInterface<transmission_interface::ActuatorToJointPositionHandle>"
  * Using a new type:
  "transmission_interface::ActuatorToJointPositionInterface"
  - Added error message printing to tests for manual inspection.

0.4.0 (2013-06-25)
------------------
* Version 0.4.0
* 1.0.1
* Update Doxygen examples with recent API changes.
* Update README.md
  Move examples out of readme and into ros_control's wiki.
* Trivial doc/whitespace fix.
* Merge branch 'master' into hardware_interface_rework
  Conflicts:
  hardware_interface/CMakeLists.txt
* Leverage ResourceManager in TransmissionInterface.
  - Refs `#45 <https://github.com/davetcoleman/ros_control/issues/45>`_ and `#48 <https://github.com/davetcoleman/ros_control/issues/48>`_.
  - Leverage hardware_interface::internal::ResourceManager to implement
  TransmissionInterface more compactly and consistently.
  - Update unit tests.
* adding install targets
* adding missing manifests
* removing comment
* merging CMakeLists.txt files from rosbuild and catkin
* adding hybrid-buildsystem makefiles
* Harmonize how variables are quoted in logs.
  - Unify to using 'single quotes'.
  - Fixes `#42 <https://github.com/davetcoleman/ros_control/issues/42>`_.
* catkinizing, could still be cleaned up
* Group transmission types in a Doxygen module.
* Rename TransmissionException class.
  Rename TransmissionException to TransmissionInterfaceException. It is more
  verbose, but more consistent with the existing HardwareInterfaceException.
* Add additional minimal example to mainpage doc.
  Existing example was complete, but quite long. It's better to start with a
  small and simple example.
* Update README.md
  Add additional minimal example.
* Update package wiki URL.
* Update README.md
* Update README.md
* Trivial doc fix.
* Add main page to documentation.
  It includes an overview of the transmission_interface package, pointers to the
  more relevant classes, and a commented example.
* Make transmission interface more general.
  The previous API assumed that to map a variable like position, one only
  needed actuator and joint space position variables. Although this is often the
  case (eg. fully actuated/determined transmissions), this does not hold in
  general. Underactuated transmissions are a typical example of this.
  Now each map accepts full <position,velocity,effort> triplets for actuator and
  joint space variables, and uses only the ones it needs.
  Although the current API has gained in generality, it has lost some of the
  explicitness it had before. For instance, if only position variables are
  needed for a map, one still needs to pass the full triplet (velocity and
  effort variables can be empty).
  Finally, unit tests and documentation have been updated to reflect the changes.
* Minor documentation building fixes.
  - Remove test folder from docs.
  - Add proper export element in manifest.
* Update transmission_interface/README.md
* Update transmission_interface/README.md
* Add readme file.
* Remove pure virtual method.
* Use \name commands in documentation.
* Add pthread dependency to tests.
  After moving from Ubuntu 10.04 to 12.04 these dependencies need to be explicitly
  stated in my dev machine. This should be looked upon in greater detail, as such
  dependecies should be taken care of by rosbuild.
* Remove dependency from manifest.
* Add transmission interface class and test.
* Add transmission accessors test.
* Remove unnecessary virtual keywords.
* Add credit statement in docs.
* Add comprehensive doc to implemented transmissions.
  - More desriptive overview.
  - Images depicting each transmission type. Binary pngs  are under version control
  instead of getting auto-generated in the Makefile as not all build environments
  may have the necessary svg->png filters.
  - Expressions governing transmissions in tabular form.
* Basic documentation for implemented transmissions.
* Document abstract Transmission class.
* Add basic support for mechanical transmissions.
  - Base transmission class with abstract interface.
  - Specializations for three common transmission types: simple, differential and
  four-bar-linkage.
  - Unit tests with exercising preconditions, black-box and white-box tests.
