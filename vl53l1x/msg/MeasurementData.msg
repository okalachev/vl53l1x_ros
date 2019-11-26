Header header

# Some of the fields of resulting RangingMeasurementData structure.

float64 signal # return signal rate in MegaCount per second (MCPS)
float64 ambient # return ambient rate in MegaCount per second (MCPS)
uint8 effective_spad # effective SPAD count for the current ranging
float64 sigma # estimation of the standard deviation of the current ranging, m
uint8 status # status for the current measurement, a value of 0 means the ranging is valid
