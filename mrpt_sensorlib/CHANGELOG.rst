^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_sensorlib
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* FIX: Implement the missing "saveToRawlog" feature
* Add parameter to set the sensorLabel of generated observations
* Limit publication of /tf sensor poses to a maximum configurable rate
* BUGFIX: tf error if sensor_frame_id==robot_frame_id
* Contributors: Jose Luis Blanco-Claraco

0.1.0 (2024-03-28)
------------------
* publish sensor pose to /tf
* fix missing namespace
* Reformat with clang-format to fix ament_linters
* Comply with ROS2 REP-2003
* Fix usage of obsolete mrpt methods
* delegate conversion to mrpt::ros2bridge
* Port to ROS2
* Contributors: Jose Luis Blanco-Claraco
