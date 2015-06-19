#ifndef DMXL_H
#define DMXL_H

#include <stdint.h>
#include "async_poll.h"
#include <stdbool.h>

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
void dmxl_process_ring(const uint_fast8_t dmxl_id);
uint8_t dmxl_ping(const uint8_t port_idx, const uint8_t dmxl_id);

void dmxl_set_torque_enable(const uint8_t port_idx, const uint8_t dmxl_id,
                            const uint8_t enable);
void dmxl_set_led(const uint8_t port_idx, const uint8_t dmxl_id,
                  const uint8_t enable);
void dmxl_set_angle_limits(const uint8_t port_idx, const uint8_t dmxl_id,
                           const uint16_t cw_limit, const uint16_t ccw_limit);
void dmxl_set_res_divider(const uint8_t port_idx, const uint8_t dmxl_id,
                          const uint8_t res_divider);
void dmxl_set_multiturn_offset(const uint8_t port_idx, const uint8_t dmxl_id,
                               const uint16_t offset);
void dmxl_set_speed_dir(const uint8_t port_idx, const uint8_t dmxl_id,
                        const uint16_t speed, const uint8_t dir);
void dmxl_set_control_mode(const uint8_t port_idx, 
                           const dmxl_control_mode_t control_mode);
void dmxl_set_control_target(const uint8_t port_idx, 
                             const uint16_t target);
void dmxl_set_all_control_targets(const uint16_t *targets);

void dmxl_poll();
void dmxl_poll_nonblocking_tick(const uint8_t dmxl_port);

typedef enum 
{ 
  DPS_WAIT,
  DPS_POLL_TX,
  DPS_POLL_RX,
  DPS_DONE = ASYNC_POLL_DONE
} dmxl_async_poll_state_t;
extern dmxl_async_poll_state_t dmxl_poll_states[NUM_DMXL];

bool dmxl_all_available();

void dmxl_set_status_return_levels();
void dmxl_set_baud_rates();

#endif 
