^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package controller_manager_tests
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.19.3 (2020-10-11)
-------------------

0.19.2 (2020-08-17)
-------------------
* Fixes part 2 of issue`#448 <https://github.com/ros-controls/ros_control/issues/448>`_ [Noetic] Rework and re-enable spawning and switching CLI tests (`#462 <https://github.com/ros-controls/ros_control/issues/462>`_)
  * [tests] Split controller_manager_scripts.txt in two parts.
  The second part is now called controller_manager_interface_test.
  This fixes part 2 of issue `#448 <https://github.com/ros-controls/ros_control/issues/448>`_
  * [test] Modification of the validation test.
  When commenting the first test (cm_msg_utils_test.py) the output
  is correct, while uncommented an inversion of the output happens.
  As this test is a pure python with no interaction with the
  controller_manager, it is very likely a race condition due either to
  Python or to the operating system. As the output is not fundamentally
  wrong, and as the tests are heavily relying on timing and subprocesses
  which are making the reproducibility difficult, this PR accepts both
  solution:
  [my_controller1, my_controller3] order
  or
  [my_controller3, my_controller1] order
* Contributors: Olivier Stasse

0.19.1 (2020-05-10)
-------------------
* Function specifiers noetic (`#453 <https://github.com/ros-controls/ros_control/issues/453>`_)
  * Add override specifiers & default constructors
  * Delete ControllerBase copy & move ctors
  * Remove unnecessary default constructors
  * Modernize additional constructors
  * Revert ImuSensorHandle::Data::Data() = default
  * Remove unnecessary default overridden constructors
  * Remove semicolon after function body
  Co-authored-by: Matt Reynolds <mtreynolds@uwaterloo.ca>
* Use setuptools instead of distutils (`#429 <https://github.com/ros-controls/ros_control/issues/429>`_)
* Contributors: Bence Magyar, Matt Reynolds

0.19.0 (2020-04-23)
-------------------
* Disable flaky CLI spawn test (`#446 <https://github.com/ros-controls/ros_control/issues/446>`_)
* Update travis config and rosinstall for noetic (`#439 <https://github.com/ros-controls/ros_control/issues/439>`_)
* Contributors: Bence Magyar

0.18.0 (2020-04-16)
-------------------
* Bump CMake version to avoid CMP0048 (`#427 <https://github.com/ros-controls/ros_control/issues/427>`_)
* Contributors: Shane Loretz

0.17.0 (2020-02-24)
-------------------
* Use auto
* Contributors: AbhinavSingh

0.16.0 (2020-01-27)
-------------------
* Merge pull request `#413 <https://github.com/ros-controls/ros_control/issues/413>`_ from matthew-reynolds/range-for
  Use range-based for loop
* Merge pull request `#404 <https://github.com/ros-controls/ros_control/issues/404>`_ from matthew-reynolds/catkin-lint
  Update CMakeLists.txt and package.xml
* Use range-based for loops in controller_manager_tests
* Update dependencies
  - Dependencies needed to compile are <build_depend>
  - Dependencies used in public headers are <build_export_depend>
  - Dependencies needed to link or run are <exec_depend>
* Merge branch 'melodic-devel' into catkin-lint
* Add ${catkin_EXPORTED_TARGETS} dependencies
* Re-install test plugin libraries
* Update package dependencies
* Re-install test plugins
* Add missing roscpp & rospy dependencies
* Correct test dependencies
* Remove installs from _tests packages
* Merge pull request `#406 <https://github.com/ros-controls/ros_control/issues/406>`_ from matthew-reynolds/pragma-once
  Use #pragma once
* Replace header guard with #pragma once
* Update package.xml descriptions
* Remove unused Boost dependencies
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
* Fix build error in clang
  error: non-aggregate type 'std::vector' (aka 'vector >') cannot be initialized with an initializer list
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
* Adapted python implementation to new parameters in SwitchController
* Adapted tests to changes in controller_manager
* fix install destination (`#377 <https://github.com/ros-controls/ros_control/issues/377>`_)
* Contributors: Bence Magyar, James Xu, Jordan Palacios, Markus Vieth, Matt Reynolds, Victor Lopez

0.15.1 (2018-09-30)
-------------------
* Added quotes for controller name and controller type in warnings and errors
* Update expected text in tests to have '' around controller names
* Contributors: Bence Magyar, Gennaro Raiola

0.15.0 (2018-05-28)
-------------------
* Add controller_group script that allows switching groups easily
* Contributors: Enrique Fernández Perdomo, Yong Li

0.14.2 (2018-04-26)
-------------------
* Update maintainers
* pluginlib: .h -> .hpp
* Fix catkin_lint errors and warnings
* Contributors: Bence Magyar

0.14.1 (2018-04-16)
-------------------
* Fix controller_manager_interface and add unit tests.
* Contributors: Yong Li

0.14.0 (2018-03-26)
-------------------
* Copyright blocks.
* Tests for extensible controllers.
* migrate to new class list macros header
* add tests for controller_manager scripts and nodes
* Contributors: Mathias Lüdtke, Mike Purvis

0.13.0 (2017-12-23)
-------------------
* Drop includes from CMake library build.
* Contributors: Mike Purvis

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
* Contributors: Bence Magyar

0.11.1 (2016-08-18)
-------------------

0.11.0 (2016-05-23)
-------------------

0.10.1 (2016-04-23)
-------------------
* Add missing test dependency on rosservice
* Remove control_toolbox dependency. Fix thread linking error coming from removal of dependency.
* Contributors: Bence Magyar

0.10.0 (2015-11-20)
-------------------
* Cleaner test exit
* Extend test suite
  - Exercise much more of the controller_manager ROS API.
  - Create multi-interface test controllers and exercise them in tests.
* Address -Wunused-parameter warnings
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.3 (2015-05-05)
------------------

0.9.2 (2015-05-04)
------------------

0.9.1 (2014-11-03)
------------------
* Update package maintainers
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.0 (2014-10-31)
------------------
* Tests for Python helpers added to controller_manager_msgs
* Buildsystem and documentation fixes
* Contributors: Adolfo Rodriguez Tsouroukdissian, Lukas Bulwahn, shadowmanos

0.8.2 (2014-06-25)
------------------

0.8.1 (2014-06-24)
------------------

0.8.0 (2014-05-12)
------------------
* controller_manager_tests: fix library linking
  From patch provided by po1 on hydro-devel.
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

0.5.8 (2013-10-11)
------------------
* Renamed manifest.xml to prevent conflicts with rosdep
* CMakeLists fix to fit with OpenEmbedded/Yocto meta-ros layer.
  Increase the compatibility of the ros_control code with
  meta-ros, an OpenEmbedded/Yocto layer that provides recipes for ROS
  packages disabling catking checking the variable CATKIN_ENABLE_TESTING.

0.5.7 (2013-07-30)
------------------

* Updated changelogs

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

0.5.0 (2013-07-16)
------------------
* Add meta tags to packages not specifying them.
  - Website, bugtracker, repository.
* adding install targets for plugin xml files
* Tests build.
* Fix build order.
* Revert "Fixed PLUGINLIB_DECLARE_CLASS deprecated errors"
  This reverts commit cd9aba265a380bafebb70d63081405d857e9380d.

0.4.0 (2013-06-25)
------------------
* Version 0.4.0
* 1.0.1
* Fixed PLUGINLIB_DECLARE_CLASS deprecated errors
* More uniform hardware_interface API. Refs  `#45 <https://github.com/davetcoleman/ros_control/issues/45>`_.
* adding install targets
* adding missing manifests
* merging CMakeLists.txt files from rosbuild and catkin
* adding hybrid-buildsystem makefiles
* catkinizing, could still be cleaned up
* port to new time api
* add wait for service
* Adding in resource/claim infrastructure
* Refactoring joint command interfaces. Also added getJointNames()
* Switching to owned interfaces, instead of multiple virtual inheritance
* Changing interface names
* Getting tests compiling again
* Fixing copyright header text
* Joint interfaces now operate on pointers, instead of refs
* test for spawning mismatched interface fails correctly
* Basic spawn test works
* Spawning dummy controller works
* Tweaking inheritance to be virtual so it compiles. dummy app with controller manager compiles
* started controller_manager_tests. untested
