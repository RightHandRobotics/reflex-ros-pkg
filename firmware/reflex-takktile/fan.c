#include "fan.h"
#include "stm32f4xx.h"

#define PORTD_FAN_EN 15

void fan_init()
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
  GPIOD->MODER |= 1 << (PORTD_FAN_EN * 2); // set as output pin
}

void fan_on()
{
  GPIOD->BSRRL |= 1 << PORTD_FAN_EN;
}

void fan_off()
{
  GPIOD->BSRRH |= 1 << PORTD_FAN_EN;
}

