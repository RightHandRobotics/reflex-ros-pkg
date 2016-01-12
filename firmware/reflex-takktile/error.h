#ifndef ERROR_H
#define ERROR_H

#include <stdint.h>

typedef enum
{
  ERR_NO_ETHERNET,     // No ethernet
  ERR_ETH_BUFFER_FULL, // The on-chip buffer for ethernet messages is full
  ERR_ETH_REMOTE_FAULT,// The PHY detected a remote fault condition
  ERR_ETH_NEGOTIATING, // The ethernet connection is being auto-negotiated
  ERR_ENC_ALL_STUCK,   // All 3 encoders have repeated the same values 100 times
  ERR_ENC_1_STUCK,     // Encoder 1 has repeated it's value
  ERR_ENC_2_STUCK,     // Encoder 2 has repeated it's value
  ERR_ENC_3_STUCK,     // Encoder 3 has repeated it's value
  ERR_TAC_0_PROBLEM,   // i2c communication error with tactile port 0
  ERR_TAC_1_PROBLEM,   // i2c communication error with tactile port 0
  ERR_TAC_2_PROBLEM,   // i2c communication error with tactile port 0
  ERR_TAC_3_PROBLEM,   // i2c communication error with tactile port 0
  ERR_NUMBER           // Placeholder to give us the number of errors, not for use
} error_type_t;

// **NOTE** Always make sure you can clear any condition you can set
void err_set(error_type_t err); // Start displaying the specific error code in the LEDs
void err_unset(error_type_t err); // Stop displaying the specific error code in the LEDs
void err_service(); // Service the LED error mechanism causing the progression of LED flashes

#endif

