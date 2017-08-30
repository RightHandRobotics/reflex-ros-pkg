#ifndef LEDS_H
#define LEDS_H

#include <stdint.h>

void leds_init();
void leds_on(uint8_t led_idx);
void leds_off(uint8_t led_idx);
void leds_toggle(uint8_t led_idx);

#endif

