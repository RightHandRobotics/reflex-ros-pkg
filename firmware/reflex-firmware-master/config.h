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
#include "spiFunc.h"
#include "takktile.h"
#include "enc.h"
#include "state.h"
#include "async_poll.h"
#include "pin.h"
#include "ports.h"
#include "enet.h"
#include "imu.h"

//TIMEOUTS
#define UART_TIMEOUT                   10000 // Microseconds [us]
#define I2C_TIMEOUT                    5000  //700  // CORRECT, put back value... <- what does this mean?
#define SPI_TIMEOUT                    5000  


// BAROMETER
#define BAROM_ADDR                     0xC0
#define BCAST_ENABLE_ADDR              0x0C
#define BCAST_DISABLE_ADDR             0x0D


// ADVANCED PERIPHERAL BUS
#define APB_MHZ                        42 	// Our APB frequency is 42 mhz


// I2C
	
// CCR: clock control register
#define I2C_CCR                        53 	
// For 100 kHz: 42 mhz / (2 * 100 khz) = 210
// For 400 kHz: 42 mhz / (2 * 400 khz) =  53

#define I2C_TRISE                      (APB_MHZ * 200 / 1000 + 1)
#define I2C_READ                       1

// FOR PRINTING INFO
#define HAND_STATE_INFO  0
#define HAND_STATUS_INFO 1


// FUNCTION PROTOTYPES
void init();
void printInfo(uint type);

#endif