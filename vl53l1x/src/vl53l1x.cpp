/*
 * STM VL53L1X ToF rangefinder driver for ROS
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * Documentation used:
 * VL53L1X datasheet - https://www.st.com/resource/en/datasheet/vl53l1x.pdf
 * VL53L1X API user manual - https://www.st.com/content/ccc/resource/technical/document/user_manual/group0/98/0d/38/38/5d/84/49/1f/DM00474730/files/DM00474730.pdf/jcr:content/translations/en.DM00474730.pdf
 *
 */

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <vl53l1x/MeasurementData.h>

#include "vl53l1_api.h"
#include "i2c.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vl53l1x");
	ros::NodeHandle nh, nh_priv("~");

	sensor_msgs::Range range;
	vl53l1x::MeasurementData data;
	range.radiation_type = sensor_msgs::Range::INFRARED;
	ros::Publisher range_pub = nh_priv.advertise<sensor_msgs::Range>("range", 20);
	ros::Publisher data_pub = nh_priv.advertise<vl53l1x::MeasurementData>("data", 20);

	// Read parameters
	int mode, i2c_bus, i2c_address;
	double poll_rate, timing_budget, offset;
	bool ignore_range_status;

	nh_priv.param("mode", mode, 3);
	nh_priv.param("i2c_bus", i2c_bus, 1);
	nh_priv.param("i2c_address", i2c_address, 0x29);
	nh_priv.param("poll_rate", poll_rate, 100.0);
	nh_priv.param("ignore_range_status", ignore_range_status, false);
	nh_priv.param("timing_budget", timing_budget, 0.1);
	nh_priv.param("offset", offset, 0.0);
	nh_priv.param<std::string>("frame_id", range.header.frame_id, "");
	nh_priv.param("field_of_view", range.field_of_view, 0.471239f); // 27 deg, source: datasheet
	nh_priv.param("min_range", range.min_range, 0.0f);
	nh_priv.param("max_range", range.max_range, 4.0f);

	if (timing_budget < 0.02 || timing_budget > 1) {
		ROS_FATAL("Error: timing_budget should be within 0.02 and 1 s (%g is set)", timing_budget);
		ros::shutdown();
	}

	// The minimum inter-measurement period must be longer than the timing budget + 4 ms (*)
	double inter_measurement_period = timing_budget + 0.004;

	// Setup I2C bus
	i2c_setup(i2c_bus, i2c_address);

	// Init sensor
	VL53L1_Dev_t dev;
	VL53L1_Error dev_error;
	VL53L1_software_reset(&dev);
	VL53L1_WaitDeviceBooted(&dev);
	VL53L1_DataInit(&dev);
	VL53L1_StaticInit(&dev);

	// Setup sensor
	VL53L1_SetXTalkCompensationEnable(&dev, 0); // Disable crosstalk compensation
	VL53L1_SetDeviceAddress(&dev, i2c_address << 1);
	VL53L1_SetDistanceMode(&dev, mode);
	VL53L1_SetMeasurementTimingBudgetMicroSeconds(&dev, round(timing_budget * 1e6));

	// Start sensor
	for (int i = 0; i < 100; i++) {
		VL53L1_SetInterMeasurementPeriodMilliSeconds(&dev, round(inter_measurement_period * 1e3));
		dev_error = VL53L1_StartMeasurement(&dev);
		if (dev_error == VL53L1_ERROR_INVALID_PARAMS) {
			inter_measurement_period += 0.001; // Increase inter_measurement_period to satisfy condition (*)
		} else break;
	}

	// Check for errors after start
	if (dev_error != VL53L1_ERROR_NONE) {
		ROS_FATAL("VL53L1X: Can't start measurement: error %d", dev_error);
		ros::shutdown();
	}

	ROS_INFO("VL53L1X: ranging");

	VL53L1_RangingMeasurementData_t measurement_data;

	// Main loop
	ros::Rate r(poll_rate);
	while (ros::ok()) {
		r.sleep();
		range.header.stamp = ros::Time::now();

		// Check the data is ready
		uint8_t data_ready = 0;
		VL53L1_GetMeasurementDataReady(&dev, &data_ready);
		if (!data_ready) {
			continue;
		}

		// Read measurement
		VL53L1_GetRangingMeasurementData(&dev, &measurement_data);
		VL53L1_ClearInterruptAndStartMeasurement(&dev);

		// Check measurement for validness
		if (!ignore_range_status && measurement_data.RangeStatus != VL53L1_RANGESTATUS_RANGE_VALID) {
			char range_status[VL53L1_MAX_STRING_LENGTH];
			VL53L1_get_range_status_string(measurement_data.RangeStatus, range_status);
			ROS_DEBUG("Range measurement status is not valid: %s", range_status);
			ros::spinOnce();
			continue;
		}

		// Publish measurement
		range.range = measurement_data.RangeMilliMeter / 1000.0 + offset;
		range_pub.publish(range);

		// Publish measurement data
		data.header.stamp = range.header.stamp;
		data.signal = measurement_data.SignalRateRtnMegaCps / 65536.0;
		data.ambient = measurement_data.AmbientRateRtnMegaCps / 65536.0;
		data.effective_spad = measurement_data.EffectiveSpadRtnCount / 256;
		data.sigma = measurement_data.SigmaMilliMeter / 65536.0 / 1000.0;
		data.status = measurement_data.RangeStatus;
		data_pub.publish(data);

		ros::spinOnce();
	}

	// Release
	ROS_INFO("VL53L1X: stop ranging");
	VL53L1_StopMeasurement(&dev);
	i2c_release();
}
