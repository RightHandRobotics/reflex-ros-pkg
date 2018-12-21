#ifndef LEDS_H
#define LEDS_H

#include <stdint.h>

#define ON                                         1
#define OFF                                        0

void leds_init();
void leds_on(uint8_t led_idx);
void leds_off(uint8_t led_idx);
void leds_toggle(uint8_t led_idx);
void ledStatus(uint8_t status);
void ledsPattern(int led1, int led2, int led3, int led4);

#endif

