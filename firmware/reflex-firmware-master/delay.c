#include "delay.h"
#include <stdint.h>

// TODO: tune this better on an oscilloscope

void delay_ns(uint32_t ns)
{
  for (volatile uint32_t i = 0; i < ns/10; i++) { }
}

void delay_us(uint32_t us)
{
  for (volatile int i = 0; i < us*10; i++) { }
}

void delay_ms(uint32_t ms)
{
  for (volatile int i = 0; i < ms; i++)
    delay_us(1000);
}

