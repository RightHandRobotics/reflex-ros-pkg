#ifndef CONSOLE_H
#define CONSOLE_H

#include "./stm32/stm32f4xx.h"
#include "systime.h"
#include "config.h"

void consoleInit();
int consolePrint(const uint8_t *buf, uint32_t len);

#endif
