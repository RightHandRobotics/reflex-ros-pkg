#include "leds.h"
#include "console.h"
#include "enet.h"
#include <stdio.h>
#include "systime.h"
#include "dmxl.h"
#include "fan.h"
#include "tactile.h"
#include "enc.h"
#include "state.h"

//#define PRINT_TACTILE_TIMING

int main()
{
  console_init();
  systime_init();
  printf("=== RESET ===\r\n");
  leds_init();
  leds_on(0);
  enet_init();
  dmxl_init();
  fan_init();
  tactile_init();
  enc_init();
  state_init();
  printf("init complete; entering main loop.\r\n");
  __enable_irq();
  /*
  dmxl_set_torque_enable(1, 2, 1);
  */
  //printf("entering main loop\r\n");
  fan_on(); // todo: be smarter. probably doesn't need to run all the time.
  for (uint_fast32_t loop_count = 1; ; loop_count++)
  {
    //if (loop_count % 2000000 == 0)
    {
      g_state.systime = SYSTIME;
      leds_toggle(0);
      leds_toggle(1);
#ifdef PRINT_TACTILE_TIMING
      volatile uint32_t t_start = SYSTIME;
#endif
      tactile_poll();
#ifdef PRINT_TACTILE_TIMING
      volatile uint32_t tactile_dt = SYSTIME - t_start;
      t_start = SYSTIME;
#endif
      enc_poll();
#ifdef PRINT_TACTILE_TIMING
      volatile uint32_t enc_dt = SYSTIME - t_start;
      printf("tactile_dt: %u  enc_dt: %u\r\n", 
             (unsigned)tactile_dt, (unsigned)enc_dt);
#endif
      dmxl_poll();
      enet_link_status_t link_status = enet_get_link_status();
      if (link_status == ENET_LINK_UP)
        enet_send_state();
    }
    enet_process_rx_ring();
    dmxl_process_rings();
  }
  return 0;
}

