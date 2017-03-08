#include "async_poll.h"

#define NUM_STATE_FUNCTIONS 14	

typedef void (*async_poll_fptr)(uint8_t poll_arg);

typedef struct
{
  const async_poll_fptr fptr;
  const uint8_t arg;
  int * const poll_state;
} stateMachine;

static uint32_t asyncStartTime = 0;

static stateMachine stateMachines[NUM_STATE_FUNCTIONS] = 
{
  { takktile_poll_nonblocking_tick, 0, (int *)(&takktilePollState[0]) },
  { takktile_poll_nonblocking_tick, 1, (int *)(&takktilePollState[1]) },
  { takktile_poll_nonblocking_tick, 2, (int *)(&takktilePollState[2]) },
  { dmxl_poll_nonblocking_tick    , 0, (int *)(&dmxl_poll_states[0])     },
  { dmxl_poll_nonblocking_tick    , 1, (int *)(&dmxl_poll_states[1])     },
  { dmxl_poll_nonblocking_tick    , 2, (int *)(&dmxl_poll_states[2])     },
  { dmxl_poll_nonblocking_tick    , 3, (int *)(&dmxl_poll_states[3])     },
  { enc_poll_nonblocking_tick     , 0, (int *)(&enc_poll_state[0])       },
  { enc_poll_nonblocking_tick     , 1, (int *)(&enc_poll_state[1])       },
  { enc_poll_nonblocking_tick     , 2, (int *)(&enc_poll_state[2])       },
  { imu_poll_nonblocking_tick     , 0, (int *)(&imu_poll_state[0])       },
  { imu_poll_nonblocking_tick     , 1, (int *)(&imu_poll_state[1])       },
  { imu_poll_nonblocking_tick     , 2, (int *)(&imu_poll_state[2])       },
  { imu_poll_nonblocking_tick     , 3, (int *)(&imu_poll_state[3])       }
}; 

void asyncInit()
{
  for (uint_fast8_t i = 0; i < NUM_STATE_FUNCTIONS; i++)
  {
    if (i < 3 || i > 6)
      *stateMachines[i].poll_state = 0;
    else
      *stateMachines[i].poll_state = ASYNC_POLL_DONE;
    stateMachines[i].fptr(stateMachines[i].arg);
  }
  asyncStartTime = SYSTIME;
}

uint8_t asyncUpdate()
{
  uint8_t allDone = 1;

  // if stateMachines did not finish before MAX_CYCLE_PERIOD, restart
  // if (SYSTIME - asyncStartTime > MAX_CYCLE_PERIOD)
  //   asyncInit();

  for (uint_fast8_t i = 0; i < NUM_STATE_FUNCTIONS; i++)
  {
    if (!(*(stateMachines[i].poll_state) == (int)ASYNC_POLL_DONE))
    {
      allDone = 0;
      stateMachines[i].fptr(stateMachines[i].arg);
    }
  }
  if (allDone)                // if all stateMachines finished
    asyncInit();              // restart

  return allDone;
}

