#ifndef ENC_H
#define ENC_H

#include <stdint.h>
#include "async_poll.h"

void enc_init();
void enc_poll();
void enc_poll_nonblocking_tick(const uint8_t bogus __attribute__((unused)));

typedef enum 
{ 
  EPS_CS_ASSERTED = 0,
  EPS_SPI_TXRX,
  EPS_SPI_TXRX_DONE,
  EPS_DONE = ASYNC_POLL_DONE
} enc_async_poll_state_t;
extern enc_async_poll_state_t enc_poll_state;

#define NUM_ENC 3

#endif

