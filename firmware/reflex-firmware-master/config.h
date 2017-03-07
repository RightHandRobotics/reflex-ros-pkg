#ifndef CONFIG_H
#define CONFIG_H

#include <stdio.h>

#include "./stm32/stm32f4xx.h"
#include "leds.h"
#include "error.h"
#include "console.h"
#include "systime.h"
#include "dmxl.h"
#include "fan.h"
#include "takktile.h"
#include "enc.h"
#include "state.h"
#include "async_poll.h"
#include "pin.h"
#include "ports.h"
#include "enet.h"
#include "imu.h"

//TIMEOUTS
#define UART_TIMEOUT                   10000 // timeout in microseconds
#define I2C_TIMEOUT                    10000  //700 // timeout in microseconds // CORRECT, put back value
#define SPI_TIMEOUT                    1000 // timeout in microseconds


// BAROMETER
#define BAROM_ADDR                     0xC0
#define BCAST_ENABLE_ADDR              0x0C
#define BCAST_DISABLE_ADDR             0x0D


#define APB_MHZ                        42 // our APB frequency is 42 mhz

// for 100 kHz i2c: 42 mhz / (2 * 100 khz) = 210
// for 400 kHz i2c: 42 mhz / (2 * 400 khz) =  53
#define I2C_CCR                        210
#define I2C_TRISE                      (APB_MHZ * 200 / 1000 + 1)
#define I2C_READ                       1

// FOR PRINTING INFO
#define HAND_STATE_INFO  0
#define HAND_STATUS_INFO 1



// FUNCTION PROTOTYPES
void init();
void printInfo(uint type);

#endif