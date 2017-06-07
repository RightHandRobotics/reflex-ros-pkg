/****************************
This file contatins code copied from https://github.com/RoboticMaterials
The copyright is unspecific.
DO NOT release this code before licensing is figured out.
****************************/

#ifndef RM_H
#define RM_H

#include "i2cFunc.h"
#include <reflex.h>
#include <stdint.h>
#include "async_poll.h"
#include "./stm32/stm32f4xx.h"
#include <stdio.h>
#include "state.h"
#include "systime.h"
#include "error.h"
#include "ports.h"

#define I2C_MULTIPLEXER_ADDRESS     (0x70)

/***** Robotic Materials CONSTANTS *****/
#define VCNL4010_ADDRESS 0x13
#define COMMAND_0 0x80  // starts measurements, relays data ready info
#define PRODUCT_ID 0x81  // product ID/revision ID, should read 0x21
#define IR_CURRENT 0x83  // sets IR current in steps of 10mA 0-200mA
#define AMBIENT_PARAMETER 0x84  // Configures ambient light measures
#define PROXIMITY_MOD 0x8F  // proximity modulator timing
#define START_PROX_MEAS 0x08
#define PROX_MEAS_RDY 0x20
#define PROX_VAL_H 0x87
#define PROX_VAL_L 0x88

#define VCNL4010_PRODUCT_ID 0x21

typedef enum 
{
  RM_STATE_START_MEAS = 0,
  RM_STATE_READ_PROX,
  RM_STATE_WAIT = ASYNC_POLL_DONE
} rm_async_poll_state_t;

extern rm_async_poll_state_t rm_poll_state[NUM_RMS];

void rm_init();
uint8_t selectMultiplexerPortRM(uint8_t port);
void rm_poll_nonblocking_tick(const uint8_t imuNumber);
uint8_t setRegisterAllRMs(uint8_t registerAddr, uint8_t data);
uint8_t setRegisterRM(int rmNumber, uint8_t registerAddr, uint8_t data);
uint8_t readRegisterRM(int rmNumber, uint8_t registerAddr, uint8_t *data);


#endif

