#include "imu.h"

// GLOBAL ALL FILES VARIABLE
imu_async_poll_state_t imu_poll_state[NUM_IMUS] = {STATE_WAIT, STATE_WAIT, STATE_WAIT, STATE_WAIT};
uint8_t imu_state_count[NUM_IMUS] = {0, 0, 0, 0};

void imuInit()
{
  uint8_t id[1] = {0};
  uint8_t result;

  // initializing imus state
  printf("initializing imu state: \n");
  for (int i = 0; i < NUM_IMUS; i++)
  {
    if (handPorts.multiplexer)
    {
      if (selectMultiplexerPort(i))
      {
          printf("\tI2C Multiplexer port %d, 0x%x: ", i, 1 << i);
      }
      else
      {
          printf("\tFailed to select I2C Multiplexer port %d\n ", i);
      }
    }
    if ((uint32_t) handPorts.imu[i] == SPI1_BASE)
    {
      result = writeRegisterSPI(handPorts.imu[i], handPorts.imuI2CAddress[i], BNO055_CHIP_ID_ADDR);
      result = readBytesSPI(handPorts.imu[i], handPorts.imuI2CAddress[i], 1, id);
    }
    else
    {
      result = writeRegisterI2C(handPorts.imu[i], handPorts.imuI2CAddress[i], BNO055_CHIP_ID_ADDR);
      result = readBytesI2C(handPorts.imu[i], handPorts.imuI2CAddress[i], 1, id);
    }
    if(*id != BNO055_ID)
    {
      printf("IMU %d not found. ID: %d, Address: 0x%x Result: %d\n", i, id[0], handPorts.imuI2CAddress[i], result);
      handStatus.imus[i] = 0;
    }
    else
    {
      printf("IMU %d found. ID: %d, Address: 0x%x Result: %d\n", i, id[0], handPorts.imuI2CAddress[i], result);
      handStatus.imus[i] = 1;
    }
  }
  // set imu mode
  printf("\tSetting modes...\n");
  result = setRegisterIMUs(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
  printf("\t\tResult: %s\n", result ? "SUCCESS" : "FAILED\n");  
  udelay(1000);

  // reset
  printf("\tReseting...\n");
  result = setRegisterIMUs(BNO055_SYS_TRIGGER_ADDR, 0x20);
  printf("\t\tResult: %s\n", result ? "SUCCESS" : "FAILED\n");  
  udelay(1000000); // takes a while to reset the imus
  
   // set imu power mode
  printf("\tSetting power modes...\n");
  result = setRegisterIMUs(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
  printf("\t\tResult: %s\n", result ? "SUCCESS" : "FAILED\n");  
  udelay(1000);
  
  // set page id
  printf("\tSetting page id...\n");
  result = setRegisterIMUs(BNO055_PAGE_ID_ADDR, 0);
  printf("\t\tResult: %s\n", result ? "SUCCESS" : "FAILED\n");  
  udelay(1000);
  
  // set external crystal use id
  printf("\tSetting external crystal use...\n");
  result = setRegisterIMUs(BNO055_SYS_TRIGGER_ADDR, 0x80);
  printf("\t\tResult: %s\n", result ? "SUCCESS" : "FAILED\n");  
  udelay(1000000);//500000); // takes a while to set this configuration
  
  // set imu mode again
  printf("\tSetting modes 0x%02x... \n", OPERATION_MODE_NDOF);
  result = setRegisterIMUs(BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF);
  printf("\t\tResult: %s\n", result ? "SUCCESS" : "FAILED\n");  
}

uint8_t setRegisterIMUs(uint8_t registerAddr, uint8_t data)
{
  uint8_t result = 0;
  uint8_t resultOp = 0;
  uint8_t response[1] = {0};
  printf("\t\tRegister: 0x%02x Data: 0x%02x\n", registerAddr, data);
  for (int i = 0; i < NUM_IMUS; i++)
  {
    if (handPorts.multiplexer)
    {
      if (selectMultiplexerPort(i))
      {
        printf("\t\tIMU on I2C Multiplexer port %d.\n", i);
      }
      else
      {
        printf("\t\tFailed to select port %d.\n", i);
      }
    }

    if ((uint32_t) handPorts.imu[i] == SPI1_BASE)
    {
      setRegisterSPI(handPorts.imu[i], handPorts.imuI2CAddress[i], registerAddr, data);
      resultOp = writeRegisterSPI(handPorts.imu[i], handPorts.imuI2CAddress[i], registerAddr);
    }
    else
    {
      setRegisterI2C(handPorts.imu[i], handPorts.imuI2CAddress[i], registerAddr, data);
      resultOp = writeRegisterI2C(handPorts.imu[i], handPorts.imuI2CAddress[i], registerAddr);
    }
    
    if (registerAddr != BNO055_SYS_TRIGGER_ADDR) // if not a reset command, check
    {
      if ((uint32_t) handPorts.imu[i] == SPI1_BASE)
      {
        readBytesSPI(handPorts.imu[i], handPorts.imuI2CAddress[i], 1, response);
      }
      else
      {
        readBytesI2C(handPorts.imu[i], handPorts.imuI2CAddress[i], 1, response);
      }
      result += (response[0] == data);
    }
    else
    {
      result += resultOp;
    }
  }
  return result == NUM_IMUS;
}

uint8_t selectMultiplexerPort(uint8_t port)
{
  if ((uint32_t) handPorts.imu[port] == SPI1_BASE)
  {
    printf("SELECTING IMU MULTIPLEXER PORT by SPI WRITE_REGISTER: IMU_NUM %d, PORT %d, ADDR 0x70, REG_ADDR %x",
      port, (int)handPorts.imu[port], 1 << port);
    return writeRegisterSPI(handPorts.imu[port], I2C_MULTIPLEXER_ADDRESS, 1 << port);
  }
  else
  {
    printf("SELECTING IMU MULTIPLEXER PORT by I2C WRITE_REGISTER: IMU_NUM %d, PORT %d, ADDR 0x70, REG_ADDR %x",
      port, (int)handPorts.imu[port], 1 << port);
    return writeRegisterI2C(handPorts.imu[port], I2C_MULTIPLEXER_ADDRESS, 1 << port);
  }
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
  // *state = IMU_STATE_WAIT;
  printf("\nIMU NUMBER: %d\nSTATE: ", imuNumber);
  switch(*state)
  {
    case IMU_STATE_SET_REGISTER:
      imu_state_count[imuNumber]++;
      printf("IMU_STATE_SET_REGISTER\n");
      if (imu_state_count[imuNumber]>3){    // Prevent the hand from getting stuck in a loop when IMU connection is lost
        // imu_fail_state_count[imuNumber]++;
        *state = IMU_STATE_READ_VALUES;  //agang addition
        // if (imu_fail_state_count[imuNumber]>20){
        //   imuInit(imuNumber);
        //   imu_fail_state_count[imuNumber] = 0;
        // }
      }
      else{
        if (handPorts.multiplexer)
          selectMultiplexerPort(imuNumber);
        // if (writeRegisterI2C(handPorts.imu[imuNumber], I2C_MULTIPLEXER_ADDRESS, 1 << imuNumber))
        //   printf("Selecting I2C Multiplexer port %d.\n", imuNumber);
        // else
        //   printf("Failed to select port %d\n", imuNumber);
        // printf("Setting Register for imu %d.\n", imuNumber);

        // BNO055_EULER_H_LSB_ADDR
        // BNO055_QUATERNION_DATA_W_LSB_ADDR
        
        if ((uint32_t) handPorts.imu[imuNumber] == SPI1_BASE)
        {
          printf("SPI_WRITING_TO_REGISTER by I2C WRITE: IMU Number %d, PORT %d, ADDR %x, REG_ADDR %x", 
          imuNumber, (int)handPorts.imu[imuNumber], handPorts.imuI2CAddress[imuNumber], BNO055_QUATERNION_DATA_W_LSB_ADDR);
          if (writeRegisterSPI(handPorts.imu[imuNumber], handPorts.imuI2CAddress[imuNumber], BNO055_QUATERNION_DATA_W_LSB_ADDR))
            *state = IMU_STATE_READ_VALUES;
        }
        else
        {
          printf("IMU_WRITING_TO_REGISTER by I2C WRITE: IMU Number %d, PORT %d, ADDR %x, REG_ADDR %x", 
          imuNumber, (int)handPorts.imu[imuNumber], handPorts.imuI2CAddress[imuNumber], BNO055_QUATERNION_DATA_W_LSB_ADDR);
          if (writeRegisterI2C(handPorts.imu[imuNumber], handPorts.imuI2CAddress[imuNumber], BNO055_QUATERNION_DATA_W_LSB_ADDR))
            *state = IMU_STATE_READ_VALUES;
        }
      }

      break;
    case IMU_STATE_READ_VALUES:
      printf("IMU_STATE_READ_VALUES\n");
      if (handPorts.multiplexer)
        selectMultiplexerPort(imuNumber);
      // if (writeRegisterI2C(handPorts.imu[imuNumber], I2C_MULTIPLEXER_ADDRESS, 1 << imuNumber))
      //   printf("Selecting I2C Multiplexer port %d.\n", imuNumber);
      // else
      //   printf("Failed to select port %d\n", imuNumber);
      // printf("Reading IMU: %d ...", imuNumber);
      if ((uint32_t) handPorts.imu[imuNumber] == SPI1_BASE)
      {
        printf("IMU_READING_BYTES by SPI READ: IMU Number %d, PORT %d, ADDR %x, length 8. ", 
          imuNumber, (int)handPorts.imu[imuNumber], handPorts.imuI2CAddress[imuNumber]);
        if(readBytesSPI(handPorts.imu[imuNumber], handPorts.imuI2CAddress[imuNumber], 8, values))
        {
          printf("reading: ");
          for (int i = 0; i < 8; i++) // CORRECT
          {
            printf("0x%02x ", values[i]);
          }
          printf("\n");
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
          // *state = IMU_STATE_WAIT;
        }
      }
      else
      {
        printf("IMU_READING_BYTES by I2C READ: IMU Number %d, PORT %d, ADDR %x, length 8. ", 
          imuNumber, (int)handPorts.imu[imuNumber], handPorts.imuI2CAddress[imuNumber]);
        if(readBytesI2C(handPorts.imu[imuNumber], handPorts.imuI2CAddress[imuNumber], 8, values))
        {
          printf("Reading: ");
          for (int i = 0; i < 8; i++) // CORRECT
          {
            printf("0x%02x ", values[i]);
          }
          printf("\n");
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
          // *state = IMU_STATE_WAIT;
        }
      }

      *state = IMU_STATE_WAIT;
      break;
    case IMU_STATE_WAIT:
      printf("IMU_STATE_WAIT\n");
      imu_state_count[imuNumber] = 0;
      break;
    default:
      printf("UNKNOWN\n");
      *state = IMU_STATE_WAIT;
      break;
  }
}