#ifndef I2CFUNC_H
#define I2CFUNC_H

#include "./stm32/stm32f4xx.h"
#include "ports.h"
#include "systime.h"

uint8_t writeBytesI2C(uint32_t* port, uint8_t address, uint8_t* data, int len, int toggleAddress);
uint8_t writeRegisterI2C(uint32_t* port, uint8_t address, uint8_t registerAddress);
uint8_t setRegisterI2C(uint32_t* port, uint8_t address, uint8_t registerAddress, uint8_t data);
uint8_t readBytesI2C(uint32_t* port, uint8_t addresss, int numBytes, uint8_t* values);
uint8_t isBusyI2CPort(uint32_t* port);

#endif