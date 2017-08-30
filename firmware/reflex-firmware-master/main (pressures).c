#include "config.h"

#include "async_poll.h"
int main()
{
  init();
  
  // while(1)
  // {
  //   errorService();
  //   if (asyncUpdate())
  //     ethernetService();
  // }


  // uint_fast8_t index;
  uint8_t data[2] = {0x12, 0x01};
  uint8_t msg[1] = {0};
  uint8_t sleepTime = 100;
  
  uint8_t aux[1] = {0};

  while(1)
  {
    writeBytesSPI(SPI1, BCAST_ENABLE_ADDR, NULL, 0, 1);  // enable all sensors    
    udelay(sleepTime);
    writeBytesSPI(SPI1, BAROM_ADDR, data, 2, 1);         // send Start Conversion Sequence
    udelay(sleepTime); // test
    readBytesSPI(SPI1, BCAST_DISABLE_ADDR >> 1, 1, aux); // disable all sensors
    udelay(3000);                                     // wait 3ms

    uint8_t values[4] = {0, 0, 0, 0};
    uint8_t addresses[9] = {0x00, 0x02, 0x04, 0x06, 0x08, 0x10, 0x12, 0x14, 0x16};
    uint16_t pressures[9] = {0};
    // uint8_t temperatures[9] = {0};  


    printf("Pressures: ");

    for (int i = 0; i < 9; i++)
    {
      
      writeBytesSPI(SPI1, addresses[i], NULL, 0, 1); // enable sensor i
      udelay(sleepTime);

      writeBytesSPI(SPI1, BAROM_ADDR, msg, 1, 1);       // choose register 0x00
      udelay(sleepTime);

      readBytesSPI(SPI1, BAROM_ADDR >> 1, 4, values);      // read 4 bytes
      udelay(sleepTime);

      pressures[i] = 510 - (values[0]<200 ? ((uint16_t)values[0] + 255) : ((uint16_t)values[0]));
      // temperatures[i] = ((uint16_t)values[2] << 2) | (values[3] >> 6);

      printf("%3d ", pressures[i]);

      readBytesSPI(SPI1, addresses[i] >> 1, 1, aux);
      udelay(sleepTime);
    }
    printf("\n");
    udelay(10000);
  }

  return 0;
}

