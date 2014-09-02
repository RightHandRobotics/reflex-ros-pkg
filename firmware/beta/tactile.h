#ifndef TACTILE_H
#define TACTILE_H

#include "reflex.h"
#include <stdint.h>
#include "async_poll.h"

void tactile_init();
void tactile_poll(const uint_fast8_t port);
void tactile_poll_nonblocking_tick(const uint8_t tactile_port);

#define SENSORS_PER_FINGER  9
#define NUM_PALM_SENSORS   11
#define NUM_SENSORS (NUM_FINGERS * SENSORS_PER_FINGER + NUM_PALM_SENSORS)
#define NUM_TACTILE_PORTS 4

typedef enum 
{ 
  TPS_IDLE = 0, 
  TPS_BCAST_ENABLE,
  TPS_BCAST_START_SAMPLING,
  TPS_BCAST_DISABLE,
  TPS_SENSOR_SAMPLING,
  TPS_SELECT_SENSOR,
  TPS_TX_READ_DATA_CMD,
  TPS_READ_DATA,
  TPS_DESELECT_SENSOR,
  TPS_DONE = ASYNC_POLL_DONE
} tactile_async_poll_state_t;

extern tactile_async_poll_state_t tactile_poll_states[NUM_TACTILE_PORTS];

#endif

