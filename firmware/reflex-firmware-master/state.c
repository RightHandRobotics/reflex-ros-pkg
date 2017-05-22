#include "state.h"

// GLOBAL ALL FILES VARIABLES
volatile state_t handState;
volatile status_t handStatus;

void state_init()
{
	// initializing handState
	handState.header[0] = 0x01; // version number of this state format
	handState.header[1] = 0x00; // pad 3 bytes so we get 32 bit alignment next
	handState.header[2] = 0x00; // ditto
	handState.header[3] = 0x00; // ditto
	handState.header[4] = 0x00; // ditto
	handState.header[5] = 0x00; // ditto	
	handState.systime = 0;
	for (uint_fast8_t i = 0; i < NUM_SENSORS; i++)
		handState.takktile_pressures[i] = handState.takktile_temperatures[i] = 0;
	for (uint_fast8_t i = 0; i < NUM_ENC; i++)
		handState.encoders[i] = 0;
	for (uint_fast8_t i = 0; i < NUM_IMUS*4; i++)
	    handState.imus[i] = 0;
    

	// initializing handStatus
	for (uint_fast8_t i = 0; i < NUM_FINGERS; i++)
		handStatus.finger[i] = 1;
	for (uint_fast8_t i = 0; i < NUM_SENSORS; i++)
		handStatus.takktileSensor[i] = 1;
	for (uint_fast8_t i = 0; i < NUM_ENC; i++)
		handStatus.encoders[i] = 1;
	for (uint_fast8_t i = 0; i < NUM_IMUS; i++)
	    handStatus.imus[i] = 0;
    for (uint_fast8_t i = 0; i < NUM_RMS; i++)
	    handStatus.rms[i] = 0;
}

