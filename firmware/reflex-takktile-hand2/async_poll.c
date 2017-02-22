#include "async_poll.h"
#include "tactile.h"
#include "dmxl.h"
#include "enc.h"
#include <stdbool.h>
#include "systime.h"
#include <stdio.h>

#define ASYNC_POLL_NUM_PORTS 9

typedef void (*async_poll_fptr)(uint8_t poll_arg);

typedef struct
{
  const async_poll_fptr fptr;
  const uint8_t arg;
  int * const poll_state;
} poll_target_t;

static uint32_t async_poll_start_time_us = 0;
static uint_fast8_t async_poll_complete = 0;

static poll_target_t poll_targets[ASYNC_POLL_NUM_PORTS] = 
{
  { tactile_poll_nonblocking_tick, 0, (int *)(&tactile_poll_states[0]) },
  { tactile_poll_nonblocking_tick, 1, (int *)(&tactile_poll_states[1]) },
  { tactile_poll_nonblocking_tick, 2, (int *)(&tactile_poll_states[2]) },
  { tactile_poll_nonblocking_tick, 3, (int *)(&tactile_poll_states[3]) },
  { dmxl_poll_nonblocking_tick   , 0, (int *)(&dmxl_poll_states[0])    },
  { dmxl_poll_nonblocking_tick   , 1, (int *)(&dmxl_poll_states[1])    },
  { dmxl_poll_nonblocking_tick   , 2, (int *)(&dmxl_poll_states[2])    },
  { dmxl_poll_nonblocking_tick   , 3, (int *)(&dmxl_poll_states[3])    },
  { enc_poll_nonblocking_tick    , 0, (int *)(&enc_poll_state)         }
};

void async_poll_init()
{
}

async_poll_tick_result_t async_poll_tick()
{
  if (async_poll_complete)
    return APT_COMPLETE;
  bool all_done = true;
  for (uint_fast8_t i = 0; i < ASYNC_POLL_NUM_PORTS; i++)
  {
    if (*(poll_targets[i].poll_state) == (int)ASYNC_POLL_DONE)
      continue;
    all_done = false;
    poll_targets[i].fptr(poll_targets[i].arg);
  }
  if (all_done)
  {
    async_poll_complete = 1;
    return APT_JUST_FINISHED;
  }
  return APT_BUSY;
}

void async_poll_start()
{
  for (uint_fast8_t i = 0; i < ASYNC_POLL_NUM_PORTS; i++)
  {
    // force them all to the DONE state, in case somebody was stuck,
    // so their state machines get the DONE -> IDLE transition on the next tick
    *poll_targets[i].poll_state = ASYNC_POLL_DONE;
    poll_targets[i].fptr(poll_targets[i].arg); // get it started
  }
  async_poll_complete = 0;
  async_poll_start_time_us = SYSTIME;
}

