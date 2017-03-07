#include "config.h"

int main()
{
  init();
  printf("Starting...\n");
  uint8_t values[40] = {0};
  uint8_t imuNumber = 0;

    // OPERATION_MODE_CONFIG
    // OPERATION_MODE_NDOF
    uint8_t data[2] = {BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF};
    uint8_t response[1] = {0};
    // writeRegisterI2C(handPorts.imu[imuNumber], BNO055_ADDRESS_A, BNO055_OPR_MODE_ADDR);
    // writeBytesI2C(handPorts.imu[imuNumber], BNO055_ADDRESS_A, data, 2, 0);
    readBytesI2C(handPorts.imu[imuNumber], BNO055_ADDRESS_A, 1, response);

    udelay(500000);
    int i = 0;
    while (response[0] != OPERATION_MODE_NDOF)
    {
      i++;
      writeBytesI2C(handPorts.imu[imuNumber], BNO055_ADDRESS_A, data, 2, 0);
      writeRegisterI2C(handPorts.imu[imuNumber], BNO055_ADDRESS_A, BNO055_OPR_MODE_ADDR);
      readBytesI2C(handPorts.imu[imuNumber], BNO055_ADDRESS_A, 1, response);
      printf("Not yet %d\n", i);
    }

  while(1)
  {
    // errorService();

    // if (asyncUpdate())
    // {
    //   printInfo(HAND_STATE_INFO);
    //   // printf("All done, send via ethernet...\n");
    //   ethernetService();
    // }    *state = ENCODER_STATE_READ_VALUES;
    // BNO055_ACCEL_DATA_Z_LSB_ADDR
    // BNO055_EULER_H_LSB_ADDR
    // BNO055_CHIP_ID_ADDR

    // uint8_t imuNumber = 0;
    // // OPERATION_MODE_CONFIG
    // // OPERATION_MODE_NDOF
    uint8_t data[2] = {BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF};
    // writeRegisterI2C(handPorts.imu[imuNumber], BNO055_ADDRESS_A, BNO055_OPR_MODE_ADDR);
    writeBytesI2C(handPorts.imu[imuNumber], BNO055_ADDRESS_A, data, 2, 0);
    // imuInit();
    // writeBytesI2C(handPorts.imu[imuNumber], BNO055_ADDRESS_A, data, 2, 0);

    udelay(1000);
    data[0] = 0;
    writeRegisterI2C(handPorts.imu[imuNumber], BNO055_ADDRESS_A, BNO055_OPR_MODE_ADDR);
    readBytesI2C(handPorts.imu[imuNumber], BNO055_ADDRESS_A, 1, data);
    printf("IMU Mode: 0x%x, should be 0x%x\n", data[0], OPERATION_MODE_NDOF);
    
    writeRegisterI2C(handPorts.imu[imuNumber], BNO055_ADDRESS_A, BNO055_CHIP_ID_ADDR);
    printf("Reading IMU: BNO055 address: 0x%02x register: 0x%02x\n", BNO055_ADDRESS_A, BNO055_CHIP_ID_ADDR);
    udelay(1000);
    if(readBytesI2C(handPorts.imu[imuNumber], BNO055_ADDRESS_A, 40, values));
    {
      udelay(1000);
      for (int i = 0; i < 40; i++)
      {
        printf("%d: 0x%02x ", i, values[i]);
        if (i % 5  == 0)
        {
          printf("\n");
        }
      }
      printf("\n");
    }
    udelay(500000);
  }
  return 0;
}
