#ifndef DMXL_H
#define DMXL_H

#include <stdint.h>

#define NUM_DMXL 4
#define DMXL_DEFAULT_ID 1

typedef enum
{
  DMXL_CM_IDLE     = 0,
  DMXL_CM_VELOCITY = 1,
  DMXL_CM_POSITION = 2
} dmxl_control_mode_t;

void dmxl_init();
void dmxl_process_rings();
uint8_t dmxl_ping(const uint8_t port_idx, const uint8_t dmxl_id);

void dmxl_set_torque_enable(const uint8_t port_idx, const uint8_t dmxl_id,
                            const uint8_t enable);
void dmxl_set_led(const uint8_t port_idx, const uint8_t dmxl_id,
                  const uint8_t enable);
void dmxl_set_angle_limits(const uint8_t port_idx, const uint8_t dmxl_id,
                           const uint16_t cw_limit, const uint16_t ccw_limit);
void dmxl_set_speed_dir(const uint8_t port_idx, const uint8_t dmxl_id,
                        const uint16_t speed, const uint8_t dir);
void dmxl_set_control_mode(const uint8_t port_idx, 
                           const dmxl_control_mode_t control_mode);
void dmxl_set_control_target(const uint8_t port_idx, 
                             const uint16_t target);
void dmxl_poll();

#endif 
