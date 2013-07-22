^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joint_limits_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
