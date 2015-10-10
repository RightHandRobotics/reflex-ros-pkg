#ifndef STATE_H
#define STATE_H

#include <stdint.h>
#include "tactile.h"
#include "enc.h"

typedef struct
{
  uint8_t  header[4];                         // 0-3
  uint32_t systime;                           // 4-7
  uint16_t tactile_pressures[NUM_SENSORS];    // 8-83
  uint16_t tactile_temperatures[NUM_SENSORS]; // 84-159
  uint16_t encoders[NUM_ENC];                 // 160-165
  uint8_t  dynamixel_error_status[4];         // 166-169
  uint16_t dynamixel_angles[4];               // 170-177
  uint16_t dynamixel_speeds[4];
  uint16_t dynamixel_loads[4];
  uint8_t  dynamixel_voltages[4];
  uint8_t  dynamixel_temperatures[4];
} __attribute__((packed)) state_t;

extern volatile state_t g_state;

void state_init();

#endif

