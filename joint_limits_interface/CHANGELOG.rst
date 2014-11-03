^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joint_limits_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.1 (2014-11-03)
------------------

0.9.0 (2014-10-31)
------------------
* Buildsystem and documentation fixes
* Add inline keyword to free header functions
* Contributors: Adolfo Rodriguez Tsouroukdissian, Lukas Bulwahn, shadowmanos

0.8.2 (2014-06-25)
------------------
* Propagate urdfdom changes to CMakeLists.txt
  urdfdom is now standalone, so it must be find_package'd independently.
* Fix rostest, which was not being built correctly.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.8.1 (2014-06-24)
------------------
* Use upstream liburdfdom-dev package.
  Refs `ros/rosdistro#4633 <https://github.com/ros/rosdistro/issues/4633>`_.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.8.0 (2014-05-12)
------------------
* Remove rosbuild artifacts. Fix `#154 <https://github.com/ros-controls/ros_control/issues/154>`_.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.7.2 (2014-04-01)
------------------

0.7.1 (2014-03-31)
------------------
* Fix dependency specification in CMake script to allow isolated builds.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.7.0 (2014-03-28)
------------------

0.6.0 (2014-02-05)
------------------
* Updated the interface list.
* Added the PositionJointSaturationInterface and VelocitySoftLimitsInterface
  classes. There are now saturation and soft limit classes for effort-controlled,
  position-controlled, and velocity-controlled joints.
* Contributors: Jim Rothrock

0.5.8 (2013-10-11)
------------------
* Merge pull request `#121 <https://github.com/ros-controls/ros_control/issues/121>`_ from pal-robotics/hydro-devel
  Fixes for next minor release
* Added the EffortJointSaturationHandle and EffortJointSaturationInterface
  classes. They are used with joints that do not have soft limits specified in
  their URDF files.
* Minor documentation precision.
* Make position joint limits handle opn loop.
  - Lowers the entry barrier for simple robots without velocity measurements,
  poor control tracking or with a slow update rate.
* Update README.md
* Create README.md
* CMakeLists fix to fit with OpenEmbedded/Yocto meta-ros layer.
  Increase the compatibility of the ros_control code with
  meta-ros, an OpenEmbedded/Yocto layer that provides recipes for ROS
  packages disabling catking checking the variable CATKIN_ENABLE_TESTING.
* Fix license header in some files.
* Renamed joint_limits_interface manifext.xml

0.5.7 (2013-07-30)
------------------

* Updated changelogs
* Add angle_wraparound joint limit property.
  For full compatibility with MoveIt!'s joint limit specification.
  Note that we still have the extra effort and jerk specification.

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
* Fixed gtests for joint_limits_interface in catkin
* Merge pull request `#93 <https://github.com/davetcoleman/ros_control/issues/93>`_ from pal-robotics/master
  joint_limits_interface broken in Groocy and Hydro
* Fix for joint_limits tests in catkin
* Restore urdf dependencies.
  Add conditional compilation for Fuerte and Groovy+ distros.

0.5.1 (2013-07-19)
------------------

0.5.0 (2013-07-16)
------------------
* Made joint_limits_interface match hydro version number
* Removed urdf_interface dependencies
* Add meta tags to packages not specifying them.
  - Website, bugtracker, repository.
* Better documentation of YAML joint limits spec.
  - Add cross-references in doc main page.
* Documentation improvements.
  - More consistency between transmission and joint limits interfaces doc.
  - Make explicit that these interfaces are not meant to be used by controllers,
  but by the robot abstraction.
* build dependency rostest added to package.xml and rostest added to CMakeLists.txt
* Added dependency for rostest to fix build error
* Fix compiler warnings (-Wreorder)
* Minor doc structure improvements.
* Add main page to joint_limits_interface doc.
* Remove temporary file from version control.
* Add attribution for soft_limits code.
  - Soft-limits enforcing is based on a previous implementation by Willow Garage.
  Add them in the copyright holders list.
* Lower severity of log message.
* Allow unsetting limits specification from rosparam.
  - Update tests.
* Add .gitignore
* Add joint limits parsing from rosparam + unit test.
* Add max_jerk to limits specification.
* Minor maintenance fixes.
* Add documentation.
* Extensive file, namespace, class renaming.

0.4.0 (2013-06-25)
------------------
