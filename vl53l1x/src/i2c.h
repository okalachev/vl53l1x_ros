#pragma once

#include <stdbool.h>

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

void i2c_setup(uint8_t bus, uint8_t addr);

void i2c_release();

EXTERNC bool i2c_readRegisterMulti(uint16_t reg, size_t count, void *pdata);

EXTERNC bool i2c_readRegisterByte(int reg, uint8_t *pdata);

EXTERNC bool i2c_readRegisterWord(uint16_t reg, uint16_t *pdata);

EXTERNC bool i2c_writeRegisterMulti(uint16_t reg, size_t count, void *pdata);

EXTERNC bool i2c_writeRegisterByte(uint16_t reg, uint8_t value);

EXTERNC bool i2c_writeRegisterWord(uint16_t reg, uint16_t value);
