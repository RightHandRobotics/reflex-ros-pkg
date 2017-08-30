#include "config.h"

#include "async_poll.h"
int main()
{
  init();

  while(1)
  {
    printf("Test\n");
    udelay(500000);
    ledsPattern(OFF, OFF, OFF, ON);
    udelay(500000);
    ledsPattern(OFF, OFF, OFF, OFF);
  }
  
  // while(1)
  // {
  //   errorService();
  //   if (asyncUpdate())
  //     ethernetService();
  // }
  return 0;
}

