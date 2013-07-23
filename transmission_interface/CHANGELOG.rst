^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package transmission_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
