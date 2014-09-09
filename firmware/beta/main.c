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
#include "async_poll.h"

//#define PRINT_TIMING

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
  async_poll_init();
  printf("init complete; entering main loop.\r\n");
  fan_on(); // todo: be smarter. probably doesn't need to run all the time.
  __enable_irq();

  volatile uint32_t prev_start_time = SYSTIME;
  //#define POLL_PERIOD_US 100000
  #define POLL_PERIOD_US 25000

  for (uint_fast32_t loop_count = 1; ; loop_count++)
  {
    if (SYSTIME - prev_start_time >= POLL_PERIOD_US)
    {
      prev_start_time += POLL_PERIOD_US;
      g_state.systime = SYSTIME;
      leds_toggle(0);
      leds_toggle(1);
      async_poll_start();
    }
    const async_poll_tick_result_t aptr = async_poll_tick();
    if (aptr == APT_JUST_FINISHED)
    {
#ifdef PRINT_TIMING
      volatile uint32_t t1 __attribute__((unused))= SYSTIME - prev_start_time;
      printf("%lu : %lu \r\n", SYSTIME, t1);
#endif
      if (enet_get_link_status() == ENET_LINK_UP)
        enet_send_state();
      enet_process_rx_ring(); // deal with inbound messages
    }
    /*
    if (loop_count % 100000 == 0)
    {
      printf("%lu aptr = %d\n", SYSTIME, aptr);
    }
    */
  }
  return 0;
}

