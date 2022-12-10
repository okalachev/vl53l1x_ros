# vl53l1x_ros

This is STM [VL53L1X](https://www.st.com/en/imaging-and-photonics-solutions/vl53l1x.html) time-of-flight rangefinder driver for ROS. Tested on a Raspberry Pi 3 and 4 with CJMCU-531 board.

The code is based on the [STM VL53L1X API library](https://www.st.com/content/st_com/en/products/embedded-software/proximity-sensors-software/stsw-img007.html).

## Connecting

Example of I²C connecting VL53L1X module to Raspberry Pi:

<img src="raspberry-vl53l1x.png" width=500>

## Installation

### From package

For Raspberry Pi, there exist prebuilt Debian packages. For installation, get the package ([Noetic](http://packages.coex.tech/packages/buster/ros-noetic-vl53l1x/), [Melodic](http://coex.space/rpi-ros-melodic/pool/main/r/ros-melodic-vl53l1x/), [Kinetic](http://coex.space/rpi-ros-kinetic/pool/main/r/ros-kinetic-vl53l1x/)) to the Raspberry and install it with `dpkg -i <package-name>.deb`.

### Manual

1. Clone the repository into your Catkin workspace:

    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/okalachev/vl53l1x_ros.git
    ```

2. Build the package:

    ```bash
    cd ~/catkin_ws
    catkin_make -DCATKIN_WHITELIST_PACKAGES="vl53l1x"
    ```

## Quick start

Run with the default settings:

```bash
rosrun vl53l1x vl53l1x_node
```

See the ranging results:

```bash
rostopic echo /vl53l1x/range
```

See the ranging rate:

```bash
rostopic hz /vl53l1x/range
```

## Parameters

All parameters are optional. Check the [`launch`](https://github.com/okalachev/vl53l1x_ros/tree/master/vl53l1x/launch) folder for launch-file templates.

* `~i2c_bus` (*int*) – I2C bus number (default: 1).
* `~i2c_address` (*int*) – I2C address (default: 0x29).
* `~mode` (*int*) – distance mode, 1 = short, 2 = medium, 3 = long (default: 3).
* `~timing_budget` (*double*) – timing budget for measurements, *s* (default: 0.1)
* `~poll_rate` (*double*) – polling data rate, *Hz* (default: 100).
* `~ignore_range_status` (*bool*) – ignore validness of measurements (default: false).
* `~pass_statuses` (*array of int*) – measurement [statuses](vl53l1x/msg/MeasurementData.msg#L13), that considered valid (default: [0, 6, 11]).
* `~min_signal` (*double*) – minimum amplitude of the signal reflected from the target to be considered valid, *MCPS* (default: 1).
* `~max_sigma` (*double*) – maximum standard deviation of the measurement to be considered valid, *m* (default: 0.015).
* `~offset` (*float*) – offset to be automatically added to measurement value, *m* (default: 0.0).
* `~frame_id` (*string*) – frame id for output `Range` messages (default: "").
* `~field_of_view` (*float*) – field of view for output `Range` messages, *rad* (default: 0.471239).
* `~min_range` (*float*) – minimum range for output `Range` messages, *m* (default: 0.0).
* `~max_range` (*float*) – maximum range for `Range` output messages, *m* (default: 4.0).

`timing_budget` is the time VL53L1X uses for ranging. The larger this time, the more accurate is measurement and the larger is maximum distance. Timing budget can be set from *0.02 s* up to *1 s*.

* *0.02 s* is the minimum timing budget and can be used only in *Short* distance mode.
* *0.033 s* is the minimum timing budget which can work for all distance modes.
* *0.14 s* is the timing budget which allows the maximum distance of *4 m* (in *Long* distance mode).

The resulting measurement rate is *1 / (timing budget + 0.004) Hz*.

Setting `ignore_range_status` to `true` makes the node to ignore `RangeStatus` field of measurements. This may significantly improve maximum distance and rate but affects quality of measurements.

`mode` is one of three distance modes, with the timing budget of *0.1 s*, *Short*, *Medium* and *Long* modes have maximum distances of 136, 290, and 360 cm, respectively.

`~min_range` and `~max_range` don't affect any device settings and only define `min_range` and `max_range` values of the output [`Range`](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Range.html) messages.

See the [official documentation](https://www.st.com/resource/en/datasheet/vl53l1x.pdf) for more information on mode and timing budget.

## Topics

### Published

* `~range` ([*sensor_msgs/Range*](http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/Range.html)) – resulting measurement.
* `~data` ([*vl53l1x/MeasurementData*](https://github.com/okalachev/vl53l1x_ros/blob/master/vl53l1x/msg/MeasurementData.msg)) – additional data of the measurement.

## Copyright

Copyright © 2019 Oleg Kalachev.

Distributed under BSD 3-Clause License (https://opensource.org/licenses/BSD-3-Clause).
