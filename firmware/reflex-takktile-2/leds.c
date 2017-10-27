#include "leds.h"
#include "./stm32/stm32f4xx.h"
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

void ledStatus(uint8_t status)
{
  switch(status & 0xF)
  {
    case 0x0:
      ledsPattern(ON, ON, ON, ON);
      break;
    case 0x1:
      ledsPattern(ON, ON, ON, OFF);
      break;
    case 0x2:
      ledsPattern(ON, ON, OFF, ON);
      break;
    case 0x3:
      ledsPattern(ON, ON, OFF, OFF);
      break;
    case 0x4:
      ledsPattern(ON, OFF, ON, ON);
      break;
    case 0x5:
      ledsPattern(ON, OFF, ON, OFF);
      break;
    case 0x6:
      ledsPattern(ON, OFF, OFF, ON);
      break;
    case 0x7:
      ledsPattern(ON, OFF, OFF, OFF);
      break;
    case 0x8:
      ledsPattern(OFF, ON, ON, ON);
      break;
    case 0x9:
      ledsPattern(OFF, ON, ON, OFF);
      break;
    case 0xA:
      ledsPattern(OFF, ON, OFF, ON);
      break;
    case 0xB: // 1011 -> 011 3
      ledsPattern(OFF, ON, OFF, OFF);
      break;
    case 0xC:
      ledsPattern(OFF, OFF, ON, ON);
      break;
    case 0xD: // 1101 -> 101 -> 5
      ledsPattern(OFF, OFF, ON, OFF);
      break;
    case 0xE:
      ledsPattern(OFF, OFF, OFF, ON);
      break;
    case 0xF:
      ledsPattern(OFF, OFF, OFF, OFF);
      break;
    default:
      ledsPattern(OFF, OFF, OFF, OFF);
      break;
  }
}

void ledsPattern(int led1, int led2, int led3, int led4)
{
  int leds[4] = {led1, led2, led3, led4};

  for(int i=0; i<4;i++)
  {
    if (leds[i] == ON)
    {
      leds_on(i);
    }
    else
    {
      leds_off(i);
    }
  }
}

