#ifndef ASYNC_POLL_H
#define ASYNC_POLL_H

#define ASYNC_POLL_DONE 0xffffffff
#define MAX_CYCLE_PERIOD 25000 // 33333

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include "takktile.h"
#include "dmxl.h"
#include "enc.h"
#include "imu.h"
#include "systime.h"


void asyncInit();
uint8_t asyncUpdate();

#endif

