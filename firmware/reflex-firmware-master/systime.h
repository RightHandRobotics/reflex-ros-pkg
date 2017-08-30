#ifndef SYSTIME_H
#define SYSTIME_H

#include "./stm32/stm32f4xx.h"

#define SYSTIME (TIM2->CNT)

void systime_init();
void udelay(int utime);

#endif

