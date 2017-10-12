#include "config.h"

/* UART DEBUGGING
*     Go to file stubs.c and change the following line:
*
*     #define DEBUG_VIA_UART true
*
*       True if you want to debug via UART, using the 4 pins on the brainboard, implies in higher latency
*       False if you want a faster normal functioning
*/

int main()
{
  init();
  printf("Starting operation...\n");

  while(1)
  {
    errorService();

    if (asyncUpdate())
    {
      // printInfo(HAND_STATE_INFO);
      // printf("All done, send via ethernet...\n");
      ethernetService();
    }    
  }
  
  return 0;
}