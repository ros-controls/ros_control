^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rqt_controller_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.19.3 (2020-10-11)
-------------------

0.19.2 (2020-08-17)
-------------------
* Use an explicit relative import path instead of implicit. (`#471 <https://github.com/ros-controls/ros_control/issues/471>`_)
  On python3 system the implicit relative import will not work. The explicit
  notation, however, should work on python >= 2.5
* Contributors: Felix Exner

0.19.1 (2020-05-10)
-------------------
* Use setuptools instead of distutils (`#429 <https://github.com/ros-controls/ros_control/issues/429>`_)
* Fix rqt displaying and handling of 'initialized' controllers (`#450 <https://github.com/ros-controls/ros_control/issues/450>`_)
  This changes two minor behaviors:
  - Now shows 'initialized' controllers as red again
  - Now shows context menus for initialized controllers
  Fixes `ros-controls/ros_control#445 <https://github.com/ros-controls/ros_control/issues/445>`_
* Contributors: Matt Reynolds, RobertWilbrandt

0.19.0 (2020-04-23)
-------------------

0.18.0 (2020-04-16)
-------------------
* Bump CMake version to avoid CMP0048 (`#427 <https://github.com/ros-controls/ros_control/issues/427>`_)
* Contributors: Shane Loretz

0.17.0 (2020-02-24)
-------------------
* added missing controller state: 'initialised'
* Contributors: ThibaultRouillard

0.16.0 (2020-01-27)
-------------------
* Update package dependencies
* Add missing roscpp & rospy dependencies
* Update package.xml descriptions
* Apply consistent style to CMakeLists.txt files
* Apply consistent style to package.xml files
* Contributors: Bence Magyar, Matt Reynolds

0.15.1 (2018-09-30)
-------------------

0.15.0 (2018-05-28)
-------------------

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
* Convert to format2, fix dependency in cmake
* Contributors: Bence Magyar

0.11.1 (2016-08-18)
-------------------
* Qt5 migration
* Contributors: Bence Magyar

0.11.0 (2016-05-23)
-------------------

0.10.1 (2016-04-23)
-------------------

0.10.0 (2015-11-20)
-------------------
* Allow running as standalone application
* Multi-interface controllers, UI revamp
  - Make the rqt_controller_manager aware of multi-interface controllers.
  - Reduce the amount of screen real-estate used by the plugin.
  - Make main view read-only.
  - Show uninitialized controllers (fetched from parameter server) in the same
  list as stopped and running controllers.
  - Make less assumptions when finding running controller managers. Use
  existing helpers on controller_mamager_msgs.utils Python module.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.3 (2015-05-05)
------------------

0.9.2 (2015-05-04)
------------------

0.9.1 (2014-11-03)
------------------

0.9.0 (2014-10-31)
------------------

0.8.2 (2014-06-25)
------------------

0.8.1 (2014-06-24)
------------------
* Register plugin under a group. Fixes `#162 <https://github.com/pal-robotics/ros_control/issues/162>`_.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.8.0 (2014-05-12)
------------------

0.7.2 (2014-04-01)
------------------
* Add plugin resources to installation target.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.7.1 (2014-03-31)
------------------

0.7.0 (2014-03-28)
------------------

0.6.0 (2014-02-05)
------------------
* Added controller namespace detection and switching, loadable controller parameter detection and buttons for loading or starting the controller directly from the parameter server.
* Resources -> Claimed Resources column title
* Initial commit for rqt controller manager plugin.  Plugin seems functional from first tests.  Allows users to unload/load/start/stop/view available controllers.  No functionality yet exists for loading a controller from scratch.
* Contributors: Kelsey

0.5.8 (2013-10-11)
------------------

0.5.7 (2013-07-30)
------------------

0.5.6 (2013-07-29)
------------------

0.5.5 (2013-07-23 17:04)
------------------------

0.5.4 (2013-07-23 14:37)
------------------------

0.5.3 (2013-07-22 18:06)
------------------------

0.5.2 (2013-07-22 15:00)
------------------------

0.5.1 (2013-07-19)
------------------

0.5.0 (2013-07-16)
------------------

0.4.0 (2013-06-25)
------------------
