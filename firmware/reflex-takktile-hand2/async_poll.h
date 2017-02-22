#ifndef ASYNC_POLL_H
#define ASYNC_POLL_H

typedef enum
{
  APT_BUSY,
  APT_JUST_FINISHED,
  APT_COMPLETE
} async_poll_tick_result_t;

void async_poll_init();
async_poll_tick_result_t async_poll_tick();
void async_poll_start();

#define ASYNC_POLL_DONE 0xffffffff

#endif

