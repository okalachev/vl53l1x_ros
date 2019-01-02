/* Based on: https://github.com/pimoroni/vl53l1x-python */

#include "vl53l1_platform.h"
#include "vl53l1_api.h"
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "i2c.h"

VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
	return i2c_writeRegisterMulti(index, count, pdata) ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
	return i2c_readRegisterMulti(index, count, pdata) ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_WrByte(VL53L1_DEV Dev, uint16_t index, uint8_t data) {
	return i2c_writeRegisterByte(index, data) ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_WrWord(VL53L1_DEV Dev, uint16_t index, uint16_t data) {
	return i2c_writeRegisterWord(index, data) ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_WrDWord(VL53L1_DEV Dev, uint16_t index, uint32_t data) {
	VL53L1_Error Status = VL53L1_ERROR_NONE;
	return Status;
}

VL53L1_Error VL53L1_UpdateByte(VL53L1_DEV Dev, uint16_t index, uint8_t AndData, uint8_t OrData) {
	VL53L1_Error Status = VL53L1_ERROR_NONE;
	return Status;
}

VL53L1_Error VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index, uint8_t *data) {
	return i2c_readRegisterByte(index, data) ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_RdWord(VL53L1_DEV Dev, uint16_t index, uint16_t *data) {
	return i2c_readRegisterWord(index, data) ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_RdDWord(VL53L1_DEV Dev, uint16_t index, uint32_t *data) {
	VL53L1_Error Status = VL53L1_ERROR_NONE;
	return Status;
}

VL53L1_Error VL53L1_GetTickCount(
	uint32_t *ptick_count_ms)
{
	VL53L1_Error status  = VL53L1_ERROR_NONE;
	return status;
}

//#define trace_print(level, ...) \
//	_LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_PLATFORM, \
//	level, VL53L1_TRACE_FUNCTION_NONE, ##__VA_ARGS__)

//#define trace_i2c(...) \
//	_LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_NONE, \
//	VL53L1_TRACE_LEVEL_NONE, VL53L1_TRACE_FUNCTION_I2C, ##__VA_ARGS__)

VL53L1_Error VL53L1_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
	VL53L1_Error status  = VL53L1_ERROR_NONE;
	return status;
}

VL53L1_Error VL53L1_WaitMs(VL53L1_Dev_t *pdev, int32_t wait_ms){
	usleep(wait_ms * 1000);
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitUs(VL53L1_Dev_t *pdev, int32_t wait_us){
	usleep(wait_us);
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitValueMaskEx(
	VL53L1_Dev_t *pdev,
	uint32_t      timeout_ms,
	uint16_t      index,
	uint8_t       value,
	uint8_t       mask,
	uint32_t      poll_delay_ms)
{
	uint8_t  register_value = 0;

	VL53L1_Error status  = VL53L1_ERROR_NONE;

	int32_t attempts = timeout_ms / poll_delay_ms;

	for(int32_t x = 0; x < attempts; x++){
		status = VL53L1_RdByte(
					pdev,
					index,
					&register_value);
		if (status == VL53L1_ERROR_NONE && (register_value & mask) == value) {
			return VL53L1_ERROR_NONE;
		}
		usleep(poll_delay_ms * 1000);
	}

	return VL53L1_ERROR_TIME_OUT;
}
