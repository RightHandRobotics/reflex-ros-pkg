#ifndef TAKKTILE_H
#define TAKKTILE_H

#include "reflex.h"
#include "spiFunc.h"
#include "i2cFunc.h"

#include <stdint.h>
#include "async_poll.h"
#include "stm32f4xx.h"
#include <stdio.h>
#include "pin.h"
#include "error.h"
#include "systime.h"
#include <string.h>
#include <stdbool.h>
#include <state.h>

#define NUM_TACTILE_PORTS   4
#define NUM_INTERNAL_I2C    2
#define NUM_BRIDGED_I2C     2

/////////////////////////////////////////////////////////////////////////
// I2C SETUP
//
// i2c1 scl = pb6, alternate function 4
// i2c1 sda = pb7, alternate function 4
// i2c3 scl = pa8, alternate function 4
// i2c3 sda = pc9, alternate function 4

#define PORTB_I2C1_SCL 6
#define PORTB_I2C1_SDA 7

#define PORTA_I2C3_SCL 8
#define PORTC_I2C3_SDA 9

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

#define PORTC_I2C_BRIDGE_RESET 14
#define PORTA_BRIDGE0_CS        4
#define PORTA_BRIDGE0_MISO      6
#define PORTB_BRIDGE0_MOSI      5
#define PORTA_BRIDGE0_SCLK      5
#define PORTC_BRIDGE0_INT      15
#define PORTB_BRIDGE1_CS        9
#define PORTC_BRIDGE1_MISO      2
#define PORTC_BRIDGE1_MOSI      3
#define PORTD_BRIDGE1_SCLK      3
#define PORTA_BRIDGE1_INT       0

#define BAROM_ADDR          0xC0
#define BCAST_ENABLE_ADDR   0x0C
#define BCAST_DISABLE_ADDR  0x0D

// our APB frequency is 42 mhz
#define APB_MHZ 42

// for 100 kHz i2c: 42 mhz / (2 * 100 khz) = 210
// for 400 kHz i2c: 42 mhz / (2 * 400 khz) =  53
#define I2C_CCR 210
#define I2C_TRISE (APB_MHZ * 200 / 1000 + 1)
#define I2C_READ 1

#define SLEEP_TIME 0
#define TACTILE_I2C_SUCCESS 0xffffffff
#define TACTILE_I2C_FAIL    0xfffffffe


void takktileInit();
void resetConverter(void);
void takktile_poll_nonblocking_tick(const uint8_t takktile_port);

typedef enum
{
  STATE_ENABLE_ALL_SENSORS = 0,
  STATE_START_CONVERSION,
  STATE_DISABLE_ALL_SENSORS,
  STATE_ENABLE_SENSOR,
  STATE_SET_REGISTER,
  STATE_READ_VALUES,
  STATE_DISABLE_SENSOR,
  STATE_WAIT = ASYNC_POLL_DONE
} takktileAsyncPollState_t;

uint8_t enableAllSensors(uint8_t takktileNumber);
uint8_t startConversionSequence(uint8_t takktileNumber);
uint8_t disableAllSensors(uint8_t takktileNumber);
uint8_t enableSensor(uint8_t takktileNumber, uint8_t sensorIndex);
uint8_t setRegister(uint8_t takktileNumber);
uint8_t readValues(uint8_t takktileNumber, uint8_t sensorIndex);
uint8_t disableSensor(uint8_t takktileNumber, uint8_t sensorIndex);

extern takktileAsyncPollState_t takktilePollState[NUM_TACTILE_PORTS];

#endif

