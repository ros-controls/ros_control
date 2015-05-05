^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package controller_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

0.8.2 (2014-06-25)
------------------

0.8.1 (2014-06-24)
------------------

0.8.0 (2014-05-12)
------------------
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
* Add .gitignore file.

0.5.7 (2013-07-30)
------------------

* Updated changelogs
* Documentation fixes.
  - Tag (non)realtime methods in ControllerBase.
  - Fix incorrect param name in Controller.

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
* Reneamed Github repo in documentation to ros-controls

0.4.0 (2013-06-25)
------------------
* Version 0.4.0
* 1.0.1
* Fix package URL in package.xml
* adding install targets
* merging CMakeLists.txt files from rosbuild and catkin
* adding hybrid-buildsystem makefiles
* Fix package URLs.
* Refactor named resource management code.
  - In preparation for the explicitly typed actuators interface, code for managing
  named resources has been refactored into a separate class. This code consists
  of convenience methods wrapping a std::map container, and occur often enough
  that factoring it out to prevent duplication makes sense.
  - Code that is not part of the public API, and hence with no stability guarantees
  has been moved to the internal folder/namespace. It only affects the named
  resource management and symbol demanglind methods so far.
* catkinizing, could still be cleaned up
* Use demangled type names when available. Fixes `#36 <https://github.com/davetcoleman/ros_control/issues/36>`_.
  Type names are used in different interfaces  such as hardware_interface and
  controller_interface. When symbol demangling is available (currently gcc 3.0+),
  operate on demangled names, as they are more convenient for human reading, eg.
  hardware_interface::VelocityJointInterface
  instead of
  N18hardware_interface22VelocityJointInterfaceE
* [Trivial] Remove redundant semicolon.
* Update controller_interface docs.
  More descriptive documentation for initialization methods with two NodeHandle
  arguments.
* add option to pass in two nodehandles to a controller: one in the root of the controller manager namespace, and one in the namespace of the controller itself. This copies the behavior used by nodelets and nodes
* Fix typo in rosdoc config files.
* Adding template parameter doc
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
* new interface with time and duration
* cleanup
* Adding in resource/claim infrastructure
* clean up publishing controller state
* Switching to owned interfaces, instead of multiple virtual inheritance
* Fixing copyright header text
* Tweaking inheritance to be virtual so it compiles. dummy app with controller manager compiles
* all pkgs now ported to fuerte
* running controller with casting. Pluginlib still messed up
* add macro
* compiling version
* move joint state controller to new package
* make a dummy plugin
* untested stuff, debians are screwed up
* compiling version
* working install target
* base classes
* first catkin stuff
