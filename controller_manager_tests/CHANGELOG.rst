^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package controller_manager_tests
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.13.3 (2018-04-26)
-------------------
* Update maintainers
* Fix catkin_lint errors and warnings
* Contributors: Bence Magyar

0.13.2 (2018-04-16)
-------------------
* Fix controller_manager_interface and add unit tests.
* migrate to new class list macros header
* Contributors: Mathias Lüdtke, Yong Li

0.13.1 (2018-03-26)
-------------------
* add tests for controller_manager scripts and nodes
* return 0 in dummy_app main
* Contributors: Mathias Lüdtke

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
