^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package controller_manager_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.19.3 (2020-10-11)
-------------------

0.19.2 (2020-08-17)
-------------------

0.19.1 (2020-05-10)
-------------------
* Use setuptools instead of distutils (`#429 <https://github.com/ros-controls/ros_control/issues/429>`_)
* Contributors: Matt Reynolds

0.19.0 (2020-04-23)
-------------------

0.18.0 (2020-04-16)
-------------------
* Bump CMake version to avoid CMP0048 (`#427 <https://github.com/ros-controls/ros_control/issues/427>`_)
* Contributors: Shane Loretz

0.17.0 (2020-02-24)
-------------------

0.16.0 (2020-01-27)
-------------------
* Update package dependencies
* Apply consistent style to CMakeLists.txt files
* Apply consistent style to package.xml files
* Fix exception module
* Contributors: Bence Magyar, Jordan Palacios, Matt Reynolds, Ryohei Ueda

0.15.1 (2018-09-30)
-------------------

0.15.0 (2018-05-28)
-------------------

0.14.2 (2018-04-26)
-------------------
* Update maintainers
* Contributors: Bence Magyar

0.14.1 (2018-04-16)
-------------------

0.14.0 (2018-03-26)
-------------------

0.13.0 (2017-12-23)
-------------------

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

0.10.1 (2016-04-23)
-------------------

0.10.0 (2015-11-20)
-------------------
* Add helper to query rosparam controller configs
  There is no way to identify uninitialized controllers other than by inspecting
  the ROS parameter server, and looking for the required controller config
  structure, which is the existence of the <ctrl_name>/type parameter.
* Multi-interface controllers
  - Breaks ROS API and (internal) Python API.
  - Modify and extend ROS message definitions to allow for controllers that
  claim resources from more than one hardware interface.
  - Modify Python helpers and update the corresponding test suite to take into
  account above changes.
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
* Add Python helpers for:
  - Getting all active controller managers.
  - Determining if a namespace contains the controller manager ROS API.
  - Filtering the output of the 'list_controllers' service by
    type, name, state, hardware_interface and claimed resources.
* Contributors: Adolfo Rodriguez Tsouroukdissian

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
* Reneamed Github repo in documentation to ros-controls

0.4.0 (2013-06-25)
------------------
* Version 0.4.0
* 1.0.1
* Fix package URL in package.xml
* merging CMakeLists.txt files from rosbuild and catkin
* adding hybrid-buildsystem makefiles
* Fix package URLs.
* catkinizing, could still be cleaned up
* Adding in resource/claim infrastructure
* add state message
* clean up publishing controller state
* get rid of pr2 stuff
* Controller manager now runs with new ControllerLoader mechanism
* all pkgs now ported to fuerte
* running controller with casting. Pluginlib still messed up
* add macro
* compiling version
* working install target
* first catkin stuff
