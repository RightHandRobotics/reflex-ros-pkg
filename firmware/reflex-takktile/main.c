#include "leds.h"
#include "error.h"
#include "console.h"
#include "enet.h"
#include <stdio.h>
#include "systime.h"
#include "dmxl.h"
#include "fan.h"
#include "tactile.h"
#include "enc.h"
#include "state.h"
#include "async_poll.h"

//#define PRINT_TIMING

int main()
{
  console_init();
  systime_init();
  printf("=== RESET ===\r\n");
  leds_init();
  enet_init();
  dmxl_init();
  fan_init();
  tactile_init();
  enc_init();
  state_init();
  async_poll_init();
  printf("init complete; entering main loop.\r\n");
  fan_on(); // todo: be smarter. probably doesn't need to run all the time.
  __enable_irq();

  dmxl_set_baud_rates();
  dmxl_set_status_return_levels();

  volatile uint32_t prev_start_time = SYSTIME;
  //#define POLL_PERIOD_US 33333
  #define POLL_PERIOD_US 25000
  uint_fast8_t poll_cycles_to_skip = 0;

  for (uint_fast32_t loop_count = 1; ; loop_count++)
  {
    err_service();
    if (SYSTIME - prev_start_time >= POLL_PERIOD_US)
    {
      prev_start_time += POLL_PERIOD_US;
      if (poll_cycles_to_skip > 0)
        poll_cycles_to_skip--;
      else
      {
        g_state.systime = SYSTIME;
        async_poll_start();
      }
    }
    const async_poll_tick_result_t aptr = async_poll_tick();
    if (aptr == APT_JUST_FINISHED)
    {
#ifdef PRINT_TIMING
      volatile uint32_t t1 __attribute__((unused))= SYSTIME - prev_start_time;
      printf("%lu : %lu \r\n", SYSTIME, t1);
#endif
      if (enet_get_link_status() == ENET_LINK_UP) {
        err_unset(ERR_NO_ETHERNET);
        enet_send_state();
      } else {
        err_set(ERR_NO_ETHERNET);
      }

      enet_process_rx_ring();
#if 0

#ifdef PRINT_TIMING
      volatile uint32_t t_before_enet = SYSTIME;
#endif
      uint_fast8_t num_rx = enet_process_rx_ring();
#ifdef PRINT_TIMING
      volatile uint32_t t_after_enet = SYSTIME;
#endif
      if (num_rx) // most inbound messages require dmxl tx/rx
      {
        // if we did something with a packet, bump our next TX time up
        // by one cycle period, so we have enough time to talk to the
        // dynamixels
        //poll_cycles_to_skip = 1; // skip the next polling cycle
        //printf("proc rx ring: %d\r\n", num_rx);
#ifdef PRINT_TIMING
        volatile uint32_t t_before_dmxl = SYSTIME;
#endif
        dmxl_process_rings();
#ifdef PRINT_TIMING
        volatile uint32_t t_after_dmxl = SYSTIME;
        printf("%8u %8u %8u %8u\r\n",
               (unsigned)t_before_enet,
               (unsigned)t_after_enet,
               (unsigned)t_before_dmxl,
               (unsigned)t_after_dmxl);
#endif
      }
#endif
    }
  }
  return 0;
}

