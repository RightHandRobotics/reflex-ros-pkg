#include "leds.h"
#include "stm32f4xx.h"
#include <stdio.h>

#define PORTE_LED0 3
#define PORTE_LED1 4
#define PORTB_LED2 10
#define PORTE_LED3 15

void leds_init()
{
  RCC->AHB1ENR   |= RCC_AHB1ENR_GPIOEEN;
  GPIOE->MODER   |= (1 << (PORTE_LED0 * 2)) |
                    (1 << (PORTE_LED1 * 2)) |
                    (1 << (PORTE_LED3 * 2));

  RCC->AHB1ENR   |= RCC_AHB1ENR_GPIOBEN;
  GPIOB->MODER   |= (1 << (PORTB_LED2 * 2));
}

void leds_on(uint8_t led)
{
  if (led == 0)
    GPIOE->BSRRL = 1 << PORTE_LED0;
  else if (led == 1)
    GPIOE->BSRRL = 1 << PORTE_LED1;
  else if (led == 2)
    GPIOB->BSRRL = 1 << PORTB_LED2;
  else if (led == 3)
    GPIOE->BSRRL = 1 << PORTE_LED3;
  else
    printf("LED %d does not exist\n", led);
}

void leds_off(uint8_t led)
{
  if (led == 0)
    GPIOE->BSRRH = 1 << PORTE_LED0;
  else if (led == 1)
    GPIOE->BSRRH = 1 << PORTE_LED1;
  else if (led == 2)
    GPIOB->BSRRH = 1 << PORTB_LED2;
  else if (led == 3)
    GPIOE->BSRRH = 1 << PORTE_LED3;
  else
    printf("LED %d does not exist\n", led);
}

void leds_toggle(uint8_t led)
{
  if (led == 0)
    GPIOE->ODR ^= 1 << PORTE_LED0;
  else if (led == 1)
    GPIOE->ODR ^= 1 << PORTE_LED1;
  else if (led == 2)
    GPIOB->ODR ^= 1 << PORTB_LED2;
  else if (led == 3)
    GPIOE->ODR ^= 1 << PORTE_LED3;
  else
    printf("LED %d does not exist\n", led);
}

