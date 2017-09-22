#ifndef ERROR_H
#define ERROR_H

#include <stdint.h>

typedef enum
{
  ERR_NO_ETHERNET,      // No Ethernet
  ERR_ETH_BUFFER_FULL,  // The on-chip buffer for Ethernet messages is full
  ERR_ETH_REMOTE_FAULT, // The PHY detected a remote fault condition
  ERR_ETH_NEGOTIATING,  // The Ethernet connection is being auto-negotiated
  ERR_ENC_ALL_STUCK,    // All 3 encoders have repeated the same values 100 times
  ERR_ENC_1_STUCK,      // Encoder 1 has repeated its value
  ERR_ENC_2_STUCK,      // Encoder 2 has repeated its value
  ERR_ENC_3_STUCK,      // Encoder 3 has repeated its value
  ERR_TAC_0_PROBLEM,    // I2C communication error with tactile port 0
  ERR_TAC_1_PROBLEM,    // I2C communication error with tactile port 1
  ERR_TAC_2_PROBLEM,    // I2C communication error with tactile port 2
  ERR_TAC_3_PROBLEM,    // I2C communication error with tactile port 3
  ERR_NUMBER,           // Placeholder to give us the number of errors, not for use
  LANCE_ERROR           // For debugging
} error_type_t;

// **NOTE** Always make sure you can clear any condition you can set
void err_set(error_type_t err); // Start displaying the specific error code in the LEDs
void err_unset(error_type_t err); // Stop displaying the specific error code in the LEDs
void errorService(); // Service the LED error mechanism causing the progression of LED flashes

#endif