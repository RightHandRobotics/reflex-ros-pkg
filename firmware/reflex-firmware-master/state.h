#ifndef STATE_H
#define STATE_H

#include <stdint.h>
#include "reflex.h"

typedef struct
{
  uint8_t  header[6];                          // 0-3
  uint32_t systime;                            // 4-7
  uint16_t takktile_pressures[NUM_SENSORS];    // 8-83
  uint16_t takktile_temperatures[NUM_SENSORS]; // 84-159
  uint16_t encoders[NUM_ENC];                  // 160-165
  uint8_t  dynamixel_error_status[4];          // 166-169
  uint16_t dynamixel_angles[4];                // 170-177
  uint16_t dynamixel_speeds[4];
  uint16_t dynamixel_loads[4];
  uint8_t  dynamixel_voltages[4];
  uint8_t  dynamixel_temperatures[4];
  uint16_t imus[NUM_IMUS*4];                   // 160-165
  uint16_t rm_raw[NUM_RMS];                     //166-171
  int32_t rm_fa[NUM_RMS];                    //172-183
  uint8_t   rm_touch[NUM_RMS];                  //184-186
} __attribute__((packed)) state_t;

typedef struct
{
  uint8_t finger[NUM_FINGERS];                 // 0-2
  uint8_t takktileSensor[NUM_SENSORS];         // 3-11
  uint8_t encoders[NUM_ENC];                   // 12-14
  uint8_t imus[NUM_IMUS];
  uint8_t rms[NUM_RMS];
} __attribute__((packed)) status_t;

// Values meaning:
// finger
//     0: not responding
//     1: responding

// takktileSensor
//     0: not responding
//     1: responding

// encoders
//     0: not responding
//     1: responding

extern volatile state_t handState;
extern volatile status_t handStatus;

void state_init();

#endif

