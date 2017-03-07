#include "imu.h"

// GLOBAL ALL FILES VARIABLE
imu_async_poll_state_t imu_poll_state[3] = {STATE_WAIT, STATE_WAIT, STATE_WAIT};

void imuInit()
{
  uint8_t id[1] = {0};
  uint8_t result;

  // initializing imus state
  printf("initializing imu state: \n");
  for (int i = 0; i < NUM_IMUS; i++)
  {
    result = writeRegisterI2C(handPorts.imu[i], BNO055_ADDRESS_A, BNO055_CHIP_ID_ADDR);
    result = readBytesI2C(handPorts.imu[i], BNO055_ADDRESS_A, 1, id);
    if(*id != BNO055_ID)
    {
      printf("\tIMU %d not found. ID: %d, Address: 0x%x Result: %d\n", i, id[0], BNO055_ADDRESS_A, result);
      handStatus.imus[i] = 0;
    }
    else
    {
      printf("\tIMU %d found. ID: %d, Address: 0x%x Result: %d\n", i, id[0], BNO055_ADDRESS_A, result);
      handStatus.imus[i] = 1;
    }
  }

  // set imu mode
  result = setRegisterIMUs(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
  printf("\tSetting modes... Result: %s\n", result ? "SUCCESS" : "FAILED");
  udelay(1000);

  // reset
  result = setRegisterIMUs(BNO055_SYS_TRIGGER_ADDR, 0x20);
  printf("\tReseting... Result: %s\n", result ? "SUCCESS" : "FAILED");
  udelay(1000);
  
   // set imu power mode
  result = setRegisterIMUs(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
  printf("\tSetting power modes... Result: %s\n", result ? "SUCCESS" : "FAILED");
  udelay(1000);
  
  // set page id
  result = setRegisterIMUs(BNO055_PAGE_ID_ADDR, 0);
  printf("\tSetting page id... Result: %s\n", result ? "SUCCESS" : "FAILED");
  udelay(1000);
  
  // set external crystal use id
  result = setRegisterIMUs(BNO055_SYS_TRIGGER_ADDR, 0x80);
  printf("\tSetting external crystal use... Result: %s\n", result ? "SUCCESS" : "FAILED");
  udelay(1000);
  
  // set imu mode again
  result = setRegisterIMUs(BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF);
  printf("\tSetting modes... Result: %s\n", result ? "SUCCESS" : "FAILED");
  udelay(1000);
  
}

uint8_t setRegisterIMUs(uint8_t registerAddr, uint8_t data)
{
  uint8_t result = 0;
  for (int i = 0; i < NUM_IMUS; i++)
    result += setRegisterI2C(handPorts.imu[i], BNO055_ADDRESS_A, registerAddr, data);
  return result == NUM_IMUS;
}

/*
void imu_poll_nonblocking_tick(const uint8_t imuNumber)
  Description: Updates the state machine

  Returns: void
  
  IMU connections:
    0 -> Port 0 on the I2C Multiplexer
    1 -> Port 1 on the I2C Multiplexer
    2 -> Port 2 on the I2C Multiplexer
    3 -> Port 3 on the I2C Multiplexer
*/
void imu_poll_nonblocking_tick(const uint8_t imuNumber)
{
  imu_async_poll_state_t* state = (imu_async_poll_state_t*)&(imu_poll_state[imuNumber]);
  uint8_t values[8] = {0};

  switch(*state)
  {
    case IMU_STATE_SET_REGISTER:
      // BNO055_EULER_H_LSB_ADDR
      // BNO055_QUATERNION_DATA_W_LSB_ADDR
      if (writeRegisterI2C(handPorts.imu[imuNumber], BNO055_ADDRESS_A, BNO055_QUATERNION_DATA_W_LSB_ADDR));
        *state = ENCODER_STATE_READ_VALUES;
      break;
    case IMU_STATE_READ_VALUES:
      // printf("Reading IMU: ");
      if(readBytesI2C(handPorts.imu[imuNumber], BNO055_ADDRESS_A, 8, values));
      {
        // for (int i = 0; i < 8; i++) // CORRECT
        // {
        //   printf("0x%02x ", values[i]);
        // }
        // printf("\n");
        // for quaternions
        handState.imus[imuNumber*4] = (((uint16_t)values[1]) << 8) | ((uint16_t)values[0]);
        handState.imus[imuNumber*4 + 1] = (((uint16_t)values[3]) << 8) | ((uint16_t)values[2]);
        handState.imus[imuNumber*4 + 2] = (((uint16_t)values[5]) << 8) | ((uint16_t)values[4]);
        handState.imus[imuNumber*4 + 3] = (((uint16_t)values[7]) << 8) | ((uint16_t)values[6]);

        // for euler
        // handState.imus[imuNumber*4] = 0;
        // handState.imus[imuNumber*4 + 1] = ((int16_t)values[0]) | (((int16_t)values[1]) << 8);
        // handState.imus[imuNumber*4 + 2] = ((int16_t)values[2]) | (((int16_t)values[3]) << 8);
        // handState.imus[imuNumber*4 + 3] = ((int16_t)values[4]) | (((int16_t)values[5]) << 8);
        *state = IMU_STATE_WAIT;
      }
      break;
    case IMU_STATE_WAIT:
      break;
    default:
      *state = IMU_STATE_WAIT;
      break;
  }
}