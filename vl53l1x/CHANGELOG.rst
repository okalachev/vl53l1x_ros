^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vl53l1x
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.1 (2021-01-16)
------------------

0.4.0 (2019-12-05)
------------------
* Release 0.4.0
* Publish measurement data even if range status is not valid
* Check return status of some of vl53l1x api function calls
* Somehow SetPresetMode for autonomous remove some glitches from device
* Remove unneeded setup calls
* Print device info on startup
* Enable C++11
* Add pass_statuses parameter for passing range statuses other than 0
* Fix range status constants in measurement data message
* Implement min_signal and max_sigma parameters
* Add status meanings to measurement data message file
* Publish additional data of the measurements
* Release 0.3.0
* Remove RefSPAD calibration from initialization
* Add ignore_range_status parameter
* Merge pull request `#3 <https://github.com/deadln/vl53l1x_ros/issues/3>`_ from sfalexrog/return_value
  i2c: Always return value from non-void function
* i2c: Always return value from non-void function
* Merge pull request `#2 <https://github.com/deadln/vl53l1x_ros/issues/2>`_ from goldarte/master
  Update version to 0.2.0
* Update version to 0.2.0
* Merge pull request `#1 <https://github.com/deadln/vl53l1x_ros/issues/1>`_ from goldarte/master
  Increase inter_measurement_period if measurements don't start
* Increase inter_measurement_period if measurements don't start
* Release 0.1.0
* Fixes
* Initial commit
* Contributors: Alexey Rogachevskiy, Arthur, Oleg Kalachev
