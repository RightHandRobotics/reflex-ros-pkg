#include "config.h"

void init()
{  
  systime_init();                  // sets the timing functions, important to be before anything else that uses time
  udelay(1000000);                 // give some time for everything to power up
  
  consoleInit();                   // console for printf over UART
  portsInit();                     // microcontroller and hand configuration

  state_init();
  takktileInit();                  // takktile
  encInit();                       // encoders
  imuInit();                       // imus
  
  leds_init();
  enet_init();
  dmxl_init();
  fan_init();
  
  
  fan_on();                        // todo: be smarter. probably doesn't need to run all the time.
  __enable_irq();
  dmxl_set_baud_rates();
  dmxl_set_status_return_levels();

  

  asyncInit();                     // initiate state machine

  printf("Hand initialization ended.\n");
}

void printInfo(uint type)
{
  switch(type)
  {
    case HAND_STATE_INFO:
      printf("Encoders: 1:%5d 2:%5d 3:%5d \n", handState.encoders[0], handState.encoders[1], handState.encoders[2]);
      for (int j = 0; j < NUM_FINGERS; j++)
      {
        printf("Pressures %d: ", j);
        for (int i = 0; i < SENSORS_PER_FINGER; i++)
        {
          printf("%3d ", handState.takktile_pressures[j * SENSORS_PER_FINGER + i]);
        }
        printf("\n");
      }
      for (int i = 0; i < NUM_IMUS; i++)
      {
        const double scale = (1.0 / (1<<14));
        printf("IMU %2d: W: %6d X: %6d Y: %6d Z: %6d\n", i, handState.imus[4*i], handState.imus[4*i + 1], handState.imus[4*i + 2], handState.imus[4*i + 3]);
        printf("        W: %6.2f X: %6.2f Y: %6.2f Z: %6.2f", scale*handState.imus[4*i], scale*handState.imus[4*i + 1], scale*handState.imus[4*i + 2], scale*handState.imus[4*i + 3]);
                

        printf("\n");
      }
      
      // printf("Pressures: \n");
      // for (int i = 0; i < NUM_SENSORS; i++)
      // {
      //   printf(" %d", handState.takktile_pressures[i]);
      // }
      // printf("\n");
      break;
    case HAND_STATUS_INFO:
      printf("Hand Status:\n");
      printf("\tTakktile Sensors: \n");
      for (int j = 0; j < NUM_FINGERS; j++)
      {
        printf("\t\tFinger %d) %d: ", j + 1, handStatus.finger[j]);
        for (int i = 0; i < SENSORS_PER_FINGER; i++)
        {
          printf("%d ", handStatus.takktileSensor[j * SENSORS_PER_FINGER + i]);
        }
        printf("\n");
      }
    break;

    default:
      break;
  }  
}