#include "leds.h"
#include "stm32f4xx.h"

#define PORTE_LED0 3
#define PORTE_LED1 4

void leds_init()
{
  RCC->AHB1ENR   |= RCC_AHB1ENR_GPIOEEN;
  GPIOE->MODER   |= (1 << (PORTE_LED0 * 2)) |
                    (1 << (PORTE_LED1 * 2));
}

void leds_on(uint8_t led)
{
  if (led == 0)
    GPIOE->BSRRL = 1 << PORTE_LED0;
  else if (led == 1)
    GPIOE->BSRRL = 1 << PORTE_LED1;
}

void leds_off(uint8_t led)
{
  if (led == 0)
    GPIOE->BSRRH = 1 << PORTE_LED0;
  else if (led == 1)
    GPIOE->BSRRH = 1 << PORTE_LED1;
}

void leds_toggle(uint8_t led)
{
  if (led == 0)
    GPIOE->ODR ^= 1 << PORTE_LED0;
  else if (led == 1)
    GPIOE->ODR ^= 1 << PORTE_LED1;
}

