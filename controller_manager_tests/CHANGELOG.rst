^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package controller_manager_tests
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
