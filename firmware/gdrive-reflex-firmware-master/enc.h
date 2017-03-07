#ifndef ENC_H
#define ENC_H

#include "i2cFunc.h"
#include "spiFunc.h"

#include <reflex.h>
#include <stdint.h>
#include "async_poll.h"
#include "./stm32/stm32f4xx.h"
#include <stdio.h>
#include "pin.h"
#include "state.h"
#include "systime.h"
#include "error.h"
#include "leds.h"   //rmelo19, for debugging
#include "ports.h"

// FOR SPI
#define PORTA_BRIDGE0_CS            4

// ERRORS
#define LIGHT_LED_THRESHOLD         100 // The number of times we get the same reading before we light an LED

// ARDUINO TESTS
#define ARDUINO_7BIT_ADDR 0x20 

// AS5048 ADDRESSES
#define AS5048_7BIT_ADDRESS         0x40 // 0b10000 + ( A1 & A2 to GND) -> 0b1000000
#define AS5048_7BIT_ADDRESS_SPI     0x40 //0x42 // 0b1000010 // 0b1000011 = 0x43 //0x10 // 0b00100 -> 0b0010000 0x10 -> 0x20 0x21
#define AS5048_ADDRESSS_READ        0x81 // 0b01000001
#define AS5048_ADDRESS_WRITE        0x80 // 0b01000000
#define AS5048B_ANGLMSB_REG         0xFE // bits 0..7
#define AS5048B_ANGLLSB_REG         0xFF // bits 0..5

// EASY TO READ
#define TRUE                        1
#define FALSE                       0
#define ON                          1
#define OFF                         0

typedef enum 
{ 
  ENCODER_STATE_SET_REGISTER = 0,
  ENCODER_STATE_READ_VALUES,
  ENCODER_STATE_WAIT = ASYNC_POLL_DONE
} enc_async_poll_state_t;

extern enc_async_poll_state_t enc_poll_state[3];

uint8_t setEncoderRegister(uint8_t encoderNumber, uint8_t encoderRegister,int timeout);
uint8_t readEncoderValues(uint8_t encoderNumber, int timeout);

void encInit();
void enc_poll();
void enc_poll_nonblocking_tick(const uint8_t encoderNumber);



#endif

