#include "error.h"
#include "leds.h"
#include "systime.h"
#include <stdbool.h>

#define NUM_LEDs  4
#define NUM_FLASHES 8

// A quarter second delay between flashes means 8 flashes is 2 seconds
static const uint32_t FLASH_INTERVAL_US = 250000;
// Here are the pattern of lights used to indicate the various error conditions
static const uint8_t errSequence[ERR_NUMBER][NUM_FLASHES] = {
  {0b1000, 0b0100, 0b0010, 0b0001, 0, 0, 0, 0}, // No Ethernet
  {0b1000, 0b0100, 0b0001, 0b0010, 0, 0, 0, 0}, // Ethernet buffer full
  {0b1000, 0b0010, 0b0100, 0b0001, 0, 0, 0, 0}, // Ethernet phy remote fault
  {0b1000, 0b0010, 0b0001, 0b0100, 0, 0, 0, 0}, // Ethernet auto-negotiation not completed
  {0b0100, 0b1000, 0b0000, 0b1110, 0, 0, 0, 0}, // Encoders all stuck
  {0b0100, 0b1000, 0b0000, 0b1000, 0, 0, 0, 0}, // Encoder 1 stuck
  {0b0100, 0b1000, 0b0000, 0b0100, 0, 0, 0, 0}, // Encoder 2 stuck
  {0b0100, 0b1000, 0b0000, 0b0010, 0, 0, 0, 0}, // Encoder 3 stuck
  {0b0100, 0b0010, 0b0000, 0b1000, 0, 0, 0, 0}, // I2C error for Tac 0
  {0b0100, 0b0010, 0b0000, 0b0100, 0, 0, 0, 0}, // I2C error for Tac 1
  {0b0100, 0b0010, 0b0000, 0b0010, 0, 0, 0, 0}, // I2C error for Tac 2
  {0b0100, 0b0010, 0b0000, 0b0001, 0, 0, 0, 0}  // I2C error for Tac 3
  };

// Which errors have been recorded
static uint8_t errDetected[ERR_NUMBER] = {0};


// Displays the appropriate LED pattern for a given error at a point in its
// sequence
void setLEDs(uint8_t errorIndex, uint8_t flashIndex) {
  uint8_t pattern = errSequence[errorIndex][flashIndex];
  for (int i = 0; i < NUM_LEDs; i++) {
    if (pattern & (1 << i)) {
      leds_on(i);
    } else {
      leds_off(i);
    }
  }
}

// This displays the error codes one by one. Each dispay is made up of four
// "flashes" of LED patterns over the first second of the display with no LEDs
// in the second second of the display
void err_service() {
  // Shows that we've started a pattern of flashes and will continue until we
  // finish it
  static bool displaying = false;
  // The index of the error we are checking and possibly displaying
  static uint8_t errorIndex = 0;
  // Which flash [0-7] of the LEDs we're currently displaying.
  static uint8_t flashIndex = 0;
  // The time (in uS) when the current flash started.
  static uint32_t flashStartTime = 0;


  // We should continue with our current display
  if (displaying) {
    if (SYSTIME - flashStartTime < FLASH_INTERVAL_US) {
      return;
    }

    flashStartTime = SYSTIME;
    flashIndex++;
    if (flashIndex < NUM_FLASHES) {
      setLEDs(errorIndex, flashIndex);
      return;
    }

    // If all are stuck we don't need to then display each individual stuck encoder
    if (errorIndex == ERR_ENC_ALL_STUCK) {
      errorIndex += 4;
    } else {
      errorIndex++;
    }
    errorIndex %= ERR_NUMBER;
    displaying = false;
    return;
  }

  // If we've found a new error start displaying it
  if (errDetected[errorIndex]) {
    displaying = true;
    flashIndex = 0;
    flashStartTime = SYSTIME;
    setLEDs(errorIndex, flashIndex);
    return;
  }

  // Otherwise keep cycling and looking for an error
  errorIndex++;
  errorIndex %= ERR_NUMBER;
  return;
}

void err_set(error_type_t err) {
  errDetected[err] = 1;
}

void err_unset(error_type_t err) {
  errDetected[err] = 0;
}
