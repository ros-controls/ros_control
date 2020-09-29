^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hardware_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.18.2 (2020-08-17)
-------------------

0.18.1 (2020-05-09)
-------------------
* Add function specifiers and modernize constructors (`#430 <https://github.com/ros-controls/ros_control/issues/430>`_)
  * Add override specifiers & default constructors
  * Delete ControllerBase copy & move ctors
  * Remove unnecessary default constructors
  * Modernize additional constructors
  * Revert ImuSensorHandle::Data::Data() = default
  * Remove unnecessary default overridden constructors
  * Remove semicolon after function body
* [hardware_interface::RobotHW] doc: update read and write, fix: group names (`#444 <https://github.com/ros-controls/ros_control/issues/444>`_)
* Contributors: Franz, Matt Reynolds

0.18.0 (2020-04-16)
-------------------
* Bump CMake version to avoid CMP0048 (`#427 <https://github.com/ros-controls/ros_control/issues/427>`_)
* Contributors: Shane Loretz

0.17.0 (2020-02-24)
-------------------
* Use default member initializers
* Use braces for member initializers
* Replace boost with std
* Replace boost::ptr_vector<T> with std::vector<T*>
* Contributors: AbhinavSingh, Bence Magyar, Matt Reynolds

0.16.0 (2020-01-27)
-------------------
* Merge pull request `#413 <https://github.com/ros-controls/ros_control/issues/413>`_ from matthew-reynolds/range-for
  Use range-based for loop
* Use more meaningful pair iterator names
* Correct typo in interface_manager.h
  Co-Authored-By: Bence Magyar <bence.magyar.robotics@gmail.com>
* Merge pull request `#404 <https://github.com/ros-controls/ros_control/issues/404>`_ from matthew-reynolds/catkin-lint
  Update CMakeLists.txt and package.xml
* Use range-based for loops in hardware_interface
* Resolve Boost dependency issues
* Update dependencies
  - Dependencies needed to compile are <build_depend>
  - Dependencies used in public headers are <build_export_depend>
  - Dependencies needed to link or run are <exec_depend>
* Merge branch 'melodic-devel' into catkin-lint
* Update package dependencies
* Remove rosunit test_depend from package.xml
* Merge pull request `#405 <https://github.com/ros-controls/ros_control/issues/405>`_ from matthew-reynolds/use-nullptr
  Use nullptr
* Use nullptr in tests
* Prefer nullptr for null pointers
* Merge pull request `#406 <https://github.com/ros-controls/ros_control/issues/406>`_ from matthew-reynolds/pragma-once
  Use #pragma once
* Replace header guard with #pragma once
* Merge pull request `#395 <https://github.com/ros-controls/ros_control/issues/395>`_ from pal-robotics-forks/extend-interfaces-melodic
  Extend interfaces
* Extend joint mode interface
* Add torque sensor and absolute encoder support to transmissions and adjust tests
  Add pointer accessors for torque sensor and absoute position encoders
* Modified structures to have absolute encoder and torque sensor parameters
* Fix argument types to use enum
* hardware_interface: fix initialization order
* Created new hardware interface for switching between controller modes
* Remove unnecessary  rosunit/rostest dependencies
* Add missing Boost dependency
* Remove redundant rosconsole dependency
* Apply consistent style to CMakeLists.txt files
* Apply consistent style to package.xml files
* Merge pull request `#399 <https://github.com/ros-controls/ros_control/issues/399>`_ from mvieth/melodic-devel
  Fix compiler warnings
* Fix compiler warnings
  - Comment out unused parameters
  - Make some integer literals unsigned to avoid comparison between signed and unsigned
  - Remove unnecessary semicolons
  - Make const void return type to void
* Merge pull request `#398 <https://github.com/ros-controls/ros_control/issues/398>`_ from matthew-reynolds/revert-cmake
  Revert CMake include_directories as SYSTEM
* Revert CMake include_directories as SYSTEM
* Merge pull request `#396 <https://github.com/ros-controls/ros_control/issues/396>`_ from pal-robotics-forks/small-fixes
  Small fixes
* Fix shadowed variables
* Multi-update cycle mode switch (`#391 <https://github.com/ros-controls/ros_control/issues/391>`_)
  For more info: https://github.com/pal-robotics-forks/ros_control2/pull/5
  * Added tests for ControllerManager update
  * Mocks for controllers and controller loader in update test
  * Divided in tests with and without controllers
  * Controller state initialized in mock
  * Moved mocks to test class
  * All tests using mock class
  * Test for multiple updates in a single controller
  * Added new switchResult() function to RobotHW interface
  ControllerManager uses this function to wait for the result of the
  doSwitch() before starting the new set of controllers
  * Using ranged based loops
  * Switch is now managed in a separate function
  * Option to start controllers as soon as their joints are ready after a switch
  * Tests for controller_interface API
  * Added new STOPPED, WAITING and ABORTED states to ControllerBase
  * Split manageSwitch() into smaller functions
  * Abort pending controllers in case of switch error
  * Changed default behaviour of new switch param
  This way if it not set it will be the same behaviour as previous version
  * Added timeout parameter to switch controller
  * Removed unnecessary includes
  * Using target_include_directories for the test
  * std::all_of instead of std::count_if
  * Deleted autogenerated file
  * Adapted tests to changes in controller_manager
  * Adapted python implementation to new parameters in SwitchController
  * Added missing parameter description docstring
  * Moved all parameters used for switching to a separate object
  * Moved error messages to controller_base
  * State check functions are now const
  * Removed unnecessary comments
  * Added constants for start_asap and timeout default parameters values
* Option to start controllers as soon as their joints are ready after a switch
* Switch is now managed in a separate function
* Added new switchResult() function to RobotHW interface
  ControllerManager uses this function to wait for the result of the
  doSwitch() before starting the new set of controllers
* Contributors: Bence Magyar, Dave Coleman, Hilario Tome, Jordan Palacios, Markus Vieth, Matt Reynolds, Paul Mathieu, Victor Lopez

0.15.1 (2018-09-30)
-------------------

0.15.0 (2018-05-28)
-------------------
* boost::shared_ptr -> std::shared_ptr
* Introduce shared_ptr typedefs
* Contributors: Bence Magyar

0.14.2 (2018-04-26)
-------------------
* Update maintainers
* Fix catkin_lint errors and warnings
* Contributors: Bence Magyar

0.14.1 (2018-04-16)
-------------------

0.14.0 (2018-03-26)
-------------------

0.13.0 (2017-12-23)
-------------------
* move CheckIsResourceManager into internal namespace
* remove unused CheckIsResourceManager::value
* do not require default constructors for HardwareInterface classes
  ResourceManager-based interfaces still need a default constructor.
* get rid of warnings for functions returning no values
* test for HW interface without default constructor
* fix constness in ImuSensorHandle
* fix constness in ForceTorqueSensorHandle
* Contributors: Mathias Lüdtke

0.12.0 (2017-08-05)
-------------------

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
* Allow the InterfaceManager class to register other InterfaceManagers.
  This will make it possible to combine several RobotHW objects into a single one.
* Contributors: Toni Oliver

0.10.1 (2016-04-23)
-------------------
* Fix rosconsole errors from test build
* Contributors: Bence Magyar

0.10.0 (2015-11-20)
-------------------
* Fix doSwitch execution point
  The doSwitch method needs to be executed in the update() method,  that is, in
  the real-time path, which is where controller switching actually takes place.
  It was previously done in the switchController callback, which is non real-time.
* Introduce prepareSwitch, replacement of canSwitch
* Add InterfaceManager::getNames
  Add new method that allows to query the names of all interfaces managed by
  an InterfaceManager instance.
* Multi-interface controllers
  - C++ API break.
  - Modify ControllerInfo class to allow controllers to claim resources from
  multiple hardware interfaces.
  - Propagate changes to RobotHW::checkForConflict: Default resource ownsership
  policy is aware of controllers claiming resources from  multiple hardware
  interfaces.
  - Update and extend the corresponding test suite.
* Address -Wunused-parameter warnings
* Contributors: Adolfo Rodriguez Tsouroukdissian, Mathias Lüdtke

0.9.3 (2015-05-05)
------------------

0.9.2 (2015-05-04)
------------------
* Add HW interface switch feature
* Contributors: Mathias Lüdtke

0.9.1 (2014-11-03)
------------------
* Update package maintainers
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.0 (2014-10-31)
------------------
* Add PosVel and PosVelAcc command interfaces
* Documentation fixes
* Contributors: Igorec, shadowmanos

0.8.2 (2014-06-25)
------------------

0.8.1 (2014-06-24)
------------------

0.8.0 (2014-05-12)
------------------
* Fix doc typo.
* Remove rosbuild artifacts. Fix `#154 <https://github.com/ros-controls/ros_control/issues/154>`_.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.7.2 (2014-04-01)
------------------

0.7.1 (2014-03-31)
------------------

0.7.0 (2014-03-28)
------------------
* Add ResourceHandle typedef
* add name to anonymous objects to avoid cppcheck error
* Contributors: Daniel Pinyol, Igorec

0.6.0 (2014-02-05)
------------------
* Update interface_manager.h
  Trivial doc fix
* Add raw data accessors to actuators interface.
  Write access to the raw actuator data will be needed for automatic transmission
  loading.
* Fix doc typo.
* Migrate RobotHW class to use InterfaceManager.
* Factor out interface management parts of RobotHW.
  - Interface management is needed in the transmission_interface package as well.
  - Add new InterfaceManager internal class, with tests.
  - RobotHW remains untouched.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.5.8 (2013-10-11)
------------------
* Renamed manifest.xml to prevent conflicts with rosdep
* Move from postfix to prefix increment in loops.
  Detected by cppcheck 'postfixOperator' warning.
* CMakeLists fix to fit with OpenEmbedded/Yocto meta-ros layer.
  Increase the compatibility of the ros_control code with
  meta-ros, an OpenEmbedded/Yocto layer that provides recipes for ROS
  packages disabling catking checking the variable CATKIN_ENABLE_TESTING.

0.5.7 (2013-07-30)
------------------

* Updated changelogs
* Author/maintainer list update.

0.5.6 (2013-07-29)
------------------

0.5.5 (2013-07-23)
------------------

0.5.4 (2013-07-23)
------------------

0.5.3 (2013-07-22)
------------------

0.5.2 (2013-07-22)
------------------

0.5.1 (2013-07-19)
------------------
* Typo fix

0.5.0 (2013-07-16)
------------------
* Add meta tags to packages not specifying them.
  - Website, bugtracker, repository.
* Merged hydro-devel into master
* Fix compiler warnings (-Wreorder)
* Remove unused headers.
* Unit test sensor interfaces.
* Add default constructors to sensor handles.
* Tests build.
* Reneamed Github repo in documentation to ros-controls
* Add missing brace.
* Update sensor interfaces implementation.
  - Use resource managing classes introduced in recent hardware interface rework.
  - Conform to unified public API.
* Remove Eigen dependency from hardware_interface.
  - Expose force-torque and IMU sensor data as const pointers to the raw data.
  - Client code should wrap raw data however they prefer.
* Explicitly initialize IMU sensor handle members.
* Scrape orientation interface prototype.
* Add sensor ref frame field and capability queries.
* Add sensor reference frame field.
* First draft of sensor interfaces.
  - Force/torque (wrench)
  - Orientation
  - IMU (very crude approximation)

0.4.0 (2013-06-25)
------------------
* Version 0.4.0
* 1.0.1
* Add another convenience symbol demangling method.
  We already had:
  string foo_name = demangledTypeName<FooType>();
  which works great for typenames, but we were missing the equivalent for specific
  instances:
  FooType foo;
  string foo_name = demangledTypeName(foo);
  ...which works well for polymorphic types, returning the derived-most name.
* Fix duplicate header guard.
* Fix package URL in package.xml
* Fix compiler warning (-Wreorder).
* Restore documentation of handle parameters.
  Documentation that was previously in the interface classes before the
  hardware interface rework has been moved to the handle classes.
* Fix ResourceManager exception messages.
  - Print derived class name instead of the less descriptive and more cryptic
  base class name. Eg.
  "hardware_interface::JointCommandInterface"
  instead of
  "hardware_interface::ResourceManager<hardware_interface::JointStateHandle>"
* Trivial doc/whitespace fix.
* Merge branch 'master' into hardware_interface_rework
  Conflicts:
  hardware_interface/CMakeLists.txt
* Separate resource manager in two classes.
  - Refs `#45 <https://github.com/davetcoleman/ros_control/issues/45>`_.
  - HardwareInterface specifics (ie. resource claiming) has been factored out.
  We now have the non-polymorphic ResourceManager class for registering and
  getting handles, and the polymorphic HardwareResourceManager that
  additionally implements the HardwareInterface and takes care of resource
  claiming.
  - The above change is required if the transmission interface is to leverage
  the resource management code, but without the hardware interface specifics.
  - Move files back to the internal folder. They are building blocks of the
  public API of hardware interfaces, but should not be directly #included
  by end users, so it's best they don't share the same location as
  user-facing headers.
  - Update unit tests.
* Add missing include statement.
* Validate raw data wrapped by hardware interfaces.
  - Refs `#47 <https://github.com/davetcoleman/ros_control/issues/47>`_ and `#52 <https://github.com/davetcoleman/ros_control/issues/52>`_.
  - Initialize raw data pointers to 0 in default handle constructors, otherwise
  they evaluate to nonzero and there is no way to distinguish an uninitialized
  state (ie. dangling pointers) from a properly initialized one.
  - For non-empty handle constructors, validate input raw data, throw if invalid
  pointers are found.
  - Add assertions on handle accessors. Invalid reads will trigger the assertions
  instead of causing a segfault (in debug mode).
  - Update unit tests.
* Warn when replacing a handle/interface.
  It is legitimate to change the underlying data associated to a handle/interface
  name, but it might also be a common programming error. Having the logs reflect
  this situation would allow to spot it easily.
* Make error message more explicit in test.
  Output with ROS_ERROR_STREAM instead of std::cout
* Add RobotHW class test.
* Add virtual destructor, protected internals.
  - ResourceManager inherits from HardwareInterface, which has virtual methods,
  so a virtual destructor is required.
  - Internal members are protected instead of private.
* Unit test hardware_interfaces.
* More uniform hardware_interface API. Refs  `#45 <https://github.com/davetcoleman/ros_control/issues/45>`_.
* adding install targets
* Restore joint resource claiming!.
  It had been mistakenly removed in a previous commit.
* merging CMakeLists.txt files from rosbuild and catkin
* adding hybrid-buildsystem makefiles
* Fix package URLs.
* Fix exception throwing.
* Harmonize how variables are quoted in logs.
  - Unify to using 'single quotes'.
  - Fixes `#42 <https://github.com/davetcoleman/ros_control/issues/42>`_.
* Merge branch 'master' of https://github.com/willowgarage/ros_control
  Conflicts:
  hardware_interface/include/hardware_interface/joint_command_interface.h
* Add explicit actuator hardware interfaces.
  - These classes are similar to the existing joint equivalents, and are useful
  in setups leveraging the transmission_interface.
* Refactor named resource management code.
  - In preparation for the explicitly typed actuators interface, code for managing
  named resources has been refactored into a separate class. This code consists
  of convenience methods wrapping a std::map container, and occur often enough
  that factoring it out to prevent duplication makes sense.
  - Code that is not part of the public API, and hence with no stability guarantees
  has been moved to the internal folder/namespace. It only affects the named
  resource management and symbol demanglind methods so far.
* catkinizing, could still be cleaned up
* add accessor for command
* Remove redundant semicolons.
* Use demangled type names when available. Fixes `#36 <https://github.com/davetcoleman/ros_control/issues/36>`_.
  Type names are used in different interfaces  such as hardware_interface and
  controller_interface. When symbol demangling is available (currently gcc 3.0+),
  operate on demangled names, as they are more convenient for human reading, eg.
  hardware_interface::VelocityJointInterface
  instead of
  N18hardware_interface22VelocityJointInterfaceE
* Fix typo in rosdoc config files.
* Fixing error message in JointCommandInterface
* More documentation in hardware_interface
* Adding template parameter doc
* Changing @ commands to \ commands
* Adding lots of inline documentation, rosdoc files
  adding inline doc to robot_hw
  adding inline doc to robot_hw
  adding inline doc to robot_hw
  more doc
  more documentation
  more doc
  more doc
  more doc
  more doc
  formatting
  adding more doc groups in controller manager
  adding more doc groups in controller manager
  Adding doc for controllerspec
  adding hardware interface docs
  adding doc to joint interfaces
  adding rosdoc for controller_interface
  Adding / reformatting doc for controller interface
* Add missing explicit header dependency.
  Don't get required header transitively, but explicitly.
* Add mising roscpp dependency.
* cleanup
* move realtime tools in ros control, and create empty constructors for handles
* Doing resource conflict check on switchControllers call
* Adding in resource/claim infrastructure
* Refactoring joint command interfaces. Also added getJointNames()
* Switching to owned interfaces, instead of multiple virtual inheritance
* Changing interface names
* joint interfaces now throw on null joint value ptrs
* JointState is now JointMeasurement, to prevent naming collisions with pr2_mechanism
* Fixing copyright header text
* Joint interfaces now operate on pointers, instead of refs
* Tweaking inheritance to be virtual so it compiles. dummy app with controller manager compiles
* started controller_manager_tests. untested
* all pkgs now ported to fuerte
* hardware interface ported to fuerte
* more renaming
* new naming scheme
* running controller with casting. Pluginlib still messed up
* add macro
* running version, with latest pluginlib
* compiling version
* untested stuff, debians are screwed up
* compiling version
* first catkin stuff
