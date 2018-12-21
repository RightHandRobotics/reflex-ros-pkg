#ifndef STATE_H
#define STATE_H

#include <stdint.h>
#include "reflex.h"

// Saves state of hand in a format sent to the computer
typedef struct
{                                               // Bytes in Packet
  uint8_t  header[6];                           // 0-5
  uint32_t systime;                             // 6-9

  // Takktile 
  uint16_t takktile_pressures[NUM_SENSORS];     // 10-115  // TODO: Correct this size here and in driver code
  uint16_t takktile_temperatures[NUM_SENSORS];  // 116-221 // TODO: Correct this size here and in driver code

  // Motor encoder?
  uint16_t encoders[NUM_ENC];                   // 222-227

  // Dynamixel motor data
  uint8_t  dynamixel_error_status[4];           // 228-231
  uint16_t dynamixel_angles[4];                 // 232-239
  uint16_t dynamixel_speeds[4];                 // 240-247
  uint16_t dynamixel_loads[4];                  // 248-255
  uint8_t  dynamixel_voltages[4];               // 256-259
  uint8_t  dynamixel_temperatures[4];           // 260-263

  // IMU data
  uint16_t imus[NUM_IMUS * 4];                  // 264-295 // Quaternion data
  uint8_t imus_calibration_status[NUM_IMUS];    // 296-299
  uint16_t imus_calibration_data[NUM_IMUS * 11];// 300-387
}__attribute__((packed)) state_t;

// imus_calibration_status
//   1 byte value containing calibration status
//       bytes 0-1: Magnetometer
//       bytes 2-3: Accelerometer
//       bytes 4-5: Gyroscope
//       bytes 6-7: Full System
//   Values
//       3: Fully Calibrated
//       0: Not Calibrated

// imu_calibration_data
//   6 bytes of offsets (x, y, z) of each sensor within 1 IMU (Accelerometer, Magnetometer, Gyroscope)
//       18 bytes total of offset values per IMU sensor
//   4 bytes of radius total for the Accelerometer and Magnetometer (LSB, MSB for each)
//   22 total bytes of calibration values per IMU

// Keeps track of takktile sensor status for each finger
typedef struct
{
  uint8_t fingerStatus;                           // Overall finger status
  uint8_t takktileSensor[SENSORS_PER_FINGER];     // Individual takktile sensor status
} __attribute__((packed)) takktile_status_t;


// Keeps track of what is working or not working
typedef struct
{
  takktile_status_t takktileFinger[3];            // Status of 3 fingers and their individual sensors
  // uint8_t finger[NUM_FINGERS];                 // Status of 3 fingers
  // uint8_t takktileSensor[NUM_SENSORS];         // Status of 53 sensors
  uint8_t encoders[NUM_ENC];                      // Status of 3 encoders
  uint8_t imus[NUM_IMUS];                         // Status of 4 IMUs
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
