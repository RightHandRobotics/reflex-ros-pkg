#ifndef PORTS_H
#define PORTS_H

#include "reflex.h"
#include <stdint.h>
#include "./stm32/stm32f4xx.h"
#include "pin.h"
#include "config.h"
#include "i2cFunc.h"

/////////////////////////////////////////////////////////////////////////
// I2C SETUP
//
// i2c1 scl = pb6, alternate function 4
// i2c1 sda = pb7, alternate function 4
// i2c3 scl = pa8, alternate function 4
// i2c3 sda = pc9, alternate function 4
#define PORTB_I2C1_SCL                6
#define PORTB_I2C1_SDA                7
#define PORTA_I2C3_SCL                8
#define PORTC_I2C3_SDA                9

/////////////////////////////////////////////////////////////////////////
// SPI-TO-I2C BRIDGES SETUP
//
// bridge shared reset = pc14
// bridge0 cs = pa4
// bridge0 miso = pa6 via SPI1 on AF 5
// bridge0 mosi = pb5 via SPI1 on AF 5
// bridge0 sclk = pa5 via SPI1 on AF 5
// bridge0 int = pc15
// bridge1 cs = pb9
// bridge1 miso = pc2 via SPI2 on AF 5
// bridge1 mosi = pc3 via SPI2 on AF 5
// bridge1 sclk = pd3 via SPI2 on AF 5
// bridge1 int = pa0
#define PORTC_I2C_BRIDGE_RESET         14
#define PORTA_BRIDGE0_CS               4
#define PORTA_BRIDGE0_MISO             6
#define PORTB_BRIDGE0_MOSI             5
#define PORTA_BRIDGE0_SCLK             5
#define PORTC_BRIDGE0_INT              15
#define PORTB_BRIDGE1_CS               9
#define PORTC_BRIDGE1_MISO             2
#define PORTC_BRIDGE1_MOSI             3
#define PORTD_BRIDGE1_SCLK             3
#define PORTA_BRIDGE1_INT              0

typedef struct
{
  uint32_t *takktile[NUM_SENSORS];
  uint32_t *encoder[NUM_ENC];
  uint32_t *imu[NUM_IMUS];
  uint8_t encoderI2CAddress[NUM_ENC];
  uint8_t imuI2CAddress[NUM_IMUS];
  uint8_t multiplexer; // 0 -> no multiplexer, 1 -> multiplexer on I2C1, 3 -> multiplexer on I2C3
} ports_t;

extern volatile ports_t handPorts;

void portsInit();


#endif