Header header

# Some of the fields of resulting RangingMeasurementData structure.

float64 signal # return signal rate in MegaCount per second (MCPS)
float64 ambient # return ambient rate in MegaCount per second (MCPS)
uint8 effective_spad # effective SPAD count for the current ranging
float64 sigma # estimation of the standard deviation of the current ranging, m
uint8 status # status for the current measurement, a value of 0 means the ranging is valid

# Status field meanings:

uint8 STATUS_RANGE_VALID = 0 # The Range is valid
uint8 STATUS_SIGMA_FAIL = 1 # Sigma Fail
uint8 STATUS_SIGNAL_FAIL = 2 # Signal fail
uint8 STATUS_RANGE_VALID_MIN_RANGE_CLIPPED = 3 # Target is below minimum detection threshold
uint8 STATUS_OUTOFBOUNDS_FAIL = 4 # Phase out of valid limits - different to a wrap exit
uint8 STATUS_HARDWARE_FAIL = 5 # Hardware fail
uint8 STATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL = 6 # The Range is valid but the wraparound check has not been done
uint8 STATUS_WRAP_TARGET_FAIL = 7 # Wrapped target - no matching phase in other VCSEL period timing
uint8 STATUS_PROCESSING_FAIL = 8 # Internal algo underflow or overflow in lite ranging
uint8 STATUS_XTALK_SIGNAL_FAIL = 9 # Specific to lite ranging
uint8 STATUS_SYNCRONISATION_INT = 10 # 1st interrupt when starting ranging in back to back mode. Ignore data
uint8 STATUS_RANGE_VALID_MERGED_PULSE = 11 # All Range ok but object is result of multiple pulses merging together. Used by RQL for merged pulse detection
uint8 STATUS_TARGET_PRESENT_LACK_OF_SIGNAL = 12 # Used by RQL as different to phase fail
uint8 STATUS_MIN_RANGE_FAIL = 13 # User ROI input is not valid e.g. beyond SPAD Array
uint8 STATUS_RANGE_INVALID = 14 # lld returned valid range but negative value
uint8 STATUS_NONE = 255 # No Update
