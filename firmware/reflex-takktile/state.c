#include "state.h"

volatile state_t g_state;

void state_init()
{
  g_state.header[0] = 0x01; // version number of this state format
  g_state.header[1] = 0x00; // pad 3 bytes so we get 32 bit alignment next
  g_state.header[2] = 0x00; // ditto
  g_state.header[3] = 0x00; // ditto
  g_state.systime = 0;
  for (uint_fast8_t i = 0; i < NUM_SENSORS; i++)
    g_state.tactile_pressures[i] = g_state.tactile_temperatures[i] = 0;
  for (uint_fast8_t i = 0; i < NUM_ENC; i++)
    g_state.encoders[i] = 0;
}

