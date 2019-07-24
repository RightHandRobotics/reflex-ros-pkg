#ifndef PIN_H
#define PIN_H

#define PIN_OUTPUT_TYPE_PUSH_PULL  0
#define PIN_OUTPUT_TYPE_OPEN_DRAIN 1

void pin_set_output_type(GPIO_TypeDef *gpio, 
                         const uint8_t pin_idx,
                         const uint8_t output_type);
void pin_set_alternate_function(GPIO_TypeDef *gpio,
                                const uint8_t pin_idx,
                                const uint8_t function_idx);
void pin_set_output(GPIO_TypeDef *gpio, const uint8_t pin_idx);
void pin_set_output_level(GPIO_TypeDef *gpio, 
                          const uint8_t pin_idx, 
                          const uint8_t pin_level);

#endif
