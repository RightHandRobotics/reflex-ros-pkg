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
  while(1)
  {
    // udelay(100);
    // readEncoderSPI(SPI1, 1, SPI_TIMEOUT); // (spiPort, encoderNumber, timeout)

    // writeBytesSPI(SPI1, BCAST_DISABLE_ADDR, NULL, 0, 1); // disable all sensors
    // udelay(100);

    // printf("Send\n");

    // writeBytesSPI(SPI1, BAROM_ADDR, data, 2, 1);         // send Start Conversion Sequence
    // udelay(100);


    // udelay(3000);

    // writeBytesSPI(SPI1, 0x00, NULL, 0, 0); // enable sensor 1
    // udelay(100);

    // uint8_t msg[1] = {0};
    // writeBytesSPI(SPI1, BAROM_ADDR, msg, 1, 1);       // choose register 0x00
    // udelay(100);

    // volatile uint8_t values[4] = {0, 0, 0, 0};
    // readBytesSPI(SPI1, BAROM_ADDR >> 1, 4, values);

    // udelay(100);

    // readCommmand(SPI1, 0x00, 1);
    // udelay(100);  }



  writeBytesSPI(SPI1, BCAST_ENABLE_ADDR, NULL, 0, 1);  // enable all sensors    
  // udelay(SLEEP_TIME);
  writeBytesSPI(SPI1, BAROM_ADDR, data, 2, 1);         // send Start Conversion Sequence
  // udelay(SLEEP_TIME); // test
  // writeBytesSPI(SPI1, BCAST_DISABLE_ADDR, NULL, 0, 1); // disable all sensors
  readBytesSPI(SPI1, BCAST_DISABLE_ADDR>>1, 0, NULL);
  udelay(3000);                                     // wait 3ms
  for (int i = 7; i < 8; i++)
  {
    volatile uint8_t values[4] = {0, 0, 0, 0};
    writeBytesSPI(SPI1, takktile_sensor_addr(tp, i), NULL, 0, 1); // enable sensor i
    // udelay(SLEEP_TIME); // test
    uint8_t msg[1] = {0};
    writeBytesSPI(SPI1, BAROM_ADDR, msg, 1, 1);       // choose register 0x00
    // udelay(SLEEP_TIME); // test
    readBytesSPI(SPI1, BAROM_ADDR >> 1, 1, values);      // read 4 bytes
    // udelay(SLEEP_TIME); // test
    index = tp * SENSORS_PER_FINGER + i;
    handState.takktile_pressures   [index] = 510 - (values[0]<200 ? ((uint16_t)values[0] + 255) : ((uint16_t)values[0]));
    handState.takktile_temperatures[index] = ((uint16_t)values[2] << 2) | (values[3] >> 6);;
    // writeBytesSPI(SPI1, takktile_sensor_addr(tp, i) + 1, NULL, 0, 1); // disable sensor i
    readBytesSPI(SPI1, takktile_sensor_addr(tp, i)>>1, 0, NULL);
    // udelay(SLEEP_TIME); // test
  }



  return 0;
}

