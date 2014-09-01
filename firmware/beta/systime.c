#include "systime.h"
#include "stm32f4xx.h"

void systime_init()
{
  // TIM2 is a 32-bit counter. it just counts microseconds since power-up.
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  for (volatile int i = 0; i < 1000; i++) { } // let tim2 spin up
  TIM2->PSC = 168000000 / 2 / 1000000 - 1; // 83
  TIM2->ARR = 0xffffffff; // count as long as possible
  TIM2->EGR = TIM_EGR_UG; // load the PSC register immediately
  TIM2->CR1 = TIM_CR1_CEN; // start counter
}

