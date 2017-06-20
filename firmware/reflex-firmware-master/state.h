#ifndef STATE_H
#define STATE_H

#include <stdint.h>
#include "reflex.h"

//state_t 
//struct to save the state of the hand in a format to be sent to the computer
typedef struct
{                                               // Bytes in Packet
  uint8_t  header[6];                           // 0-5
  uint32_t systime;                             // 6-9
  uint16_t takktile_pressures[NUM_SENSORS];     // 10-115
  uint16_t takktile_temperatures[NUM_SENSORS];  // 116-221
  uint16_t encoders[NUM_ENC];                   // 222-227
  uint8_t  dynamixel_error_status[4];           // 228-231
  uint16_t dynamixel_angles[4];                 // 232-239
  uint16_t dynamixel_speeds[4];                 // 240-247
  uint16_t dynamixel_loads[4];                  // 248-255
  uint8_t  dynamixel_voltages[4];               // 256-259
  uint8_t  dynamixel_temperatures[4];           // 260-263
  uint16_t imus[NUM_IMUS*4];                    // 264-295

//status_t
//struct used to keep track of what is working or not working
typedef struct
{
  uint8_t finger[NUM_FINGERS];                 // Status of 3 fingers
  uint8_t takktileSensor[NUM_SENSORS];         // Status of 53 sensors
  uint8_t encoders[NUM_ENC];                   // Status of 3 encoders
  uint8_t imus[NUM_IMUS];                      // Status of 4 IMUs
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
