^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package controller_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.5 (2013-07-23)
------------------
* Tweaked Changelog

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
* Removed urdf_interface dependencies
* Fix spawner choke when namespace is unspecified.
  Add missing check in conditional.
* Add meta tags to packages not specifying them.
  - Website, bugtracker, repository.
* Making script install target install scripts so that they are executable
* Fix build order.
* Combined exceptions per jbohren
* Reneamed Github repo in documentation to ros-controls
* Better timeout error checking, necessary for Gazebo
* User error checking

0.4.0 (2013-06-25)
------------------
* Version 0.4.0
* 1.0.1
* Fixing failure mode in new catkin cmakelists
* Added namespace argument to spawner script
* Fix package URL in package.xml
* Python install for controller_manager.
* Fix build order dependency.
* adding install targets
* merging CMakeLists.txt files from rosbuild and catkin
* adding hybrid-buildsystem makefiles
* Fix package URLs.
* catkinizing, could still be cleaned up
* Additional log feedback when load_controller fails
  When loading a controller fails bacause its configuration was not found on the
  parameter server, show the namespace where the parameters are expected to help
  debugging.
* Remove unused method. Fixes `#33 <https://github.com/davetcoleman/ros_control/issues/33>`_.
* add option to pass in two nodehandles to a controller: one in the root of the controller manager namespace, and one in the namespace of the controller itself. This copies the behavior used by nodelets and nodes
* Fix typo in rosdoc config files.
* Adding explicit header for recursive mutex
* Removing getControllerByNameImpl
* Switching controller_manager ``controllers_lock_`` to be a recursive lock
* Fixing comment indent
* Adding template parameter doc
* Changing @ commands to \ commands
* More doc in controller manager
* Adding clearer ros warning in controller switching
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
* don't clear vectors in realtime
* Make public getControllerByName method thread-safe.
  Existing virtual non-threadsafe method has been suffixed with -Impl and pushed
  to protected class scope. In-class uses call getControllerByNameImpl, as the
  lock has already been acquired.
* new interface with time and duration
* add missing include
* remove .svn folder
* Doing resource conflict check on switchControllers call
* Adding in resource/claim infrastructure
* fix command line interface
* clean up publishing controller state
* Controller spec now also copies over type
* Switching to owned interfaces, instead of multiple virtual inheritance
* add scripts for controller manager
* get rid of pr2 stuff
* Controller manager can now register ControllerLoaders
* Controller manager now runs with new ControllerLoader mechanism
* Creating new plugin_loader interface
* Adding debugging printouts
* Namespacing controller_spec
* Fixing copyright header text
* Spawning dummy controller works
* Tweaking inheritance to be virtual so it compiles. dummy app with controller manager compiles
* all pkgs now ported to fuerte
* add missing file
* running controller with casting. Pluginlib still messed up
* add macro
* running version, with latest pluginlib
* compiling version
* compiling version
* first catkin stuff
