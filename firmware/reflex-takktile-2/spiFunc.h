#ifndef SPIFUNC_H
#define SPIFUNC_H

#include "./stm32/stm32f4xx.h"
#include "systime.h"
#include "takktile.h"
#include <stdio.h>
// #include <stdint.h>
#include "ports.h"

#define SC18IS601_I2C_CLOCK_369KHZ                 0x05
#define SC18IS601_I2C_CLOCK_263KHZ                 0x07
#define SC18IS601_I2C_CLOCK_204KHZ                 0x09
#define SC18IS601_I2C_CLOCK_97KHZ                  0x13
#define SC18IS601_I2C_CLOCK_7200HZ                 0xFF

#define SC18IS601_REGISTER_IO_CONFIG               0x00
#define SC18IS601_REGISTER_IO_STATE                0x01
#define SC18IS601_REGISTER_I2C_CLOCK               0x02
#define SC18IS601_REGISTER_I2C_TIMEOUT             0x03
#define SC18IS601_REGISTER_I2C_STATUS              0x04
#define SC18IS601_REGISTER_I2C_ADDR                0x05

#define SC18IS601_WRITE_REGISTER_COMMAND           0x20
#define SC18IS601_READ_REGISTER_COMMAND            0x21
#define SC18IS601_WRITE_N_BYTES_COMMAND            0x00
#define SC18IS601_READ_N_BYTES_COMMAND             0x01
#define SC18IS601_READ_BUFFER_COMMAND              0x06
#define PORTC_I2C_BRIDGE_RESET                     14

#define SC18IS601_REGISTER_I2C_STATUS_SUCCESS			 0xF0
#define SC18IS601_REGISTER_I2C_STATUS_NACK_RW			 0xF1
#define SC18IS601_REGISTER_I2C_STATUS_NACK_BYTE		 0xF2
#define SC18IS601_REGISTER_I2C_STATUS_BUSY				 0xF3


// SPI to I2C Converter Functions
void resetConverter(void);
uint8_t checkConverterIsBusy (uint8_t utime);
uint8_t writeConverterRegister(uint8_t registerAddress, uint8_t data);
uint8_t readConverterRegister(uint8_t registerAddress, uint8_t *data);
uint8_t converterInit(void);

//SPI Functions
uint8_t writeRegisterSPI(uint32_t* port, uint8_t address, uint8_t registerAddress);
uint8_t setRegisterSPI(uint32_t* port, uint8_t address, uint8_t registerAddress, uint8_t data);
uint8_t readBytesSPI(uint32_t* port, uint8_t address, uint8_t numBytes, uint8_t* values);
uint8_t writeBytesSPI(uint32_t* port, uint8_t address, uint8_t* data, int len, int toggleAddress);

uint8_t readCommmand(SPI_TypeDef* spiPort,uint8_t address, uint8_t numBytes);

#endif