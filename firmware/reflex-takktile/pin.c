#include "stm32f4xx.h"
#include "pin.h"
#include <stdio.h>

void pin_set_output_type(GPIO_TypeDef *gpio, 
                         const uint8_t pin_idx,
                         const uint8_t output_type)
{
  if (output_type == PIN_OUTPUT_TYPE_OPEN_DRAIN)
  {
    //printf("setting pin %d to open-drain\r\n", pin_idx);
    gpio->OTYPER |= (1 << pin_idx);
  }
  else
  {
    //printf("setting pin %d to push-pull\r\n", pin_idx);
    gpio->OTYPER &= ~(1 << pin_idx);
  }
}

void pin_set_alternate_function(GPIO_TypeDef *gpio,
                                const uint8_t pin_idx,
                                const uint8_t function_idx)
{
  if (pin_idx > 15 || function_idx > 15)
    return; // adios amigo
  volatile uint32_t *af_reg = (pin_idx < 8) ? &gpio->AFR[0] : &gpio->AFR[1];
  const uint8_t reg_ofs = (pin_idx < 8) ? (pin_idx * 4) : ((pin_idx-8) * 4);
  *af_reg &= ~(0xf << reg_ofs); // zero out whatever was there before
  *af_reg |= function_idx << reg_ofs; // set the alternate function register
  gpio->MODER &= ~(3 << (pin_idx * 2)); // zero out whatever was there before
  gpio->MODER |= 2 << (pin_idx * 2); // put the GPIO in alternate-function mode
}

void pin_set_output(GPIO_TypeDef *gpio, const uint8_t pin_idx)
{
  if (pin_idx > 15)
    return; // adios amigo
  gpio->MODER &= ~(3 << (pin_idx * 2));
  gpio->MODER |= 1 << (pin_idx * 2);
}

void pin_set_output_level(GPIO_TypeDef *gpio, 
                          const uint8_t pin_idx, 
                          const uint8_t pin_level)
{
  if (pin_idx > 15)
    return;
  if (pin_level)
    gpio->BSRRL = 1 << pin_idx;
  else
    gpio->BSRRH = 1 << pin_idx;
}

