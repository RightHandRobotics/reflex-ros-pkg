#ifndef TACTILE_H
#define TACTILE_H

#include "reflex.h"
#include <stdint.h>

void tactile_init();
void tactile_poll();

#define SENSORS_PER_FINGER  9
#define NUM_PALM_SENSORS   11
#define NUM_SENSORS (NUM_FINGERS * SENSORS_PER_FINGER + NUM_PALM_SENSORS)
#define NUM_TACTILE_PORTS 4

#endif

