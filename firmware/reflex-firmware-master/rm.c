#include "rm.h"

// GLOBAL ALL FILES VARIABLE
rm_async_poll_state_t rm_poll_state[NUM_RMS] = {STATE_WAIT, STATE_WAIT, STATE_WAIT}; //silly to have a define when still initialized manually?
uint8_t rm_state_counter[NUM_RMS] = {0, 0, 0}; //var for not getting stuck

/***** USER PARAMETERS *****/
int ir_current_ = 8;                     // range = [0, 20]. current = value * 10 mA
int ambient_light_measurement_rate_ = 7; // range = [0, 7]. 1, 2, 3, 4, 5, 6, 8, 10 samples per second
int averaging_function_ = 7;  // range [0, 7] measurements per run are 2**value, with range [1, 2**7 = 128]
int proximity_freq_ = 0; // range = [0 , 3]. 390.625kHz, 781.250kHz, 1.5625MHz, 3.125MHz
unsigned long time;

// Touch/release detection
#define EA 0.3  // exponential average weight parameter / cut-off frequency for high-pass filter

/***** GLOBAL VARIABLES *****/
unsigned int proximity_value[NUM_RMS]; // current proximity reading
unsigned int average_value[NUM_RMS];   // low-pass filtered proximity reading
signed int fa2[NUM_RMS];              // FA-II value;
signed int fa2derivative[NUM_RMS];     // Derivative of the FA-II value;
signed int fa2deriv_last[NUM_RMS];     // Last value of the derivative (for zero-crossing detection)
signed int sensitivity = 50;  // Sensitivity of touch/release detection, values closer to zero increase sensitivity

unsigned long start_time;
int continuous_mode = 1; //Default on
int single_shot = 0;
int touch_analysis = 1; //Default on

void rmInit()
{
  uint8_t id[1] = {0};
  uint8_t result;

  // initializing rms state
  printf("initializing rm state: \n");
  for (int i = 0; i < NUM_RMS; i++)
  {
    if (handPorts.multiplexer) //select multiplexer port
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
    if ((uint32_t) handPorts.rm[i] == SPI1_BASE)
    {
      result = writeRegisterSPI(handPorts.rm[i],VCNL4010_ADDRESS, PRODUCT_ID);
      result = readBytesSPI(handPorts.rm[i], VCNL4010_ADDRESS, 1, id);
    }
    else
    {
      result = writeRegisterI2C(handPorts.rm[i], VCNL4010_ADDRESS, PRODUCT_ID);
      result = readBytesI2C(handPorts.rm[i], VCNL4010_ADDRESS, 1, id);
    }
    if(id[0] != VCNL4010_PRODUCT_ID)
    {
      printf("RM %d not found. ID: %d, Address: 0x%x Result: %d\n", i, id[0], VCNL4010_ADDRESS, result);
      handStatus.rms[i] = 0;
    }
    else
    {
      printf("RM %d found. ID: %d, Address: 0x%x Result: %d\n", i, id[0], VCNL4010_ADDRESS, result);
      handStatus.rms[i] = 1;
    }
  }
  // set ambient param
  printf("\tSetting ambient param...\n");
  result = setRegisterRMs(AMBIENT_PARAMETER, 0x7F);
  printf("\t\tResult: %s\n", result ? "SUCCESS" : "FAILED\n");  
  udelay(1000);

  // set IR current
  printf("\tsetting ir current...\n");
  result = setRegisterRMs(IR_CURRENT, ir_current_);
  printf("\t\tResult: %s\n", result ? "SUCCESS" : "FAILED\n");  
  udelay(1000); // takes a while to reset the imus
  
   // seet proximity mod
  printf("\tsetting proximity mod...\n");
  result = setRegisterRMs(PROXIMITY_MOD, 1);
  printf("\t\tResult: %s\n", result ? "SUCCESS" : "FAILED\n");  
  udelay(1000);
  
  
}


uint8_t setRegisterRMs(uint8_t registerAddr, uint8_t data)
{
  uint8_t result = 0;
  uint8_t resultOp = 0;
  uint8_t response[1] = {0};
  printf("\t\tRegister: 0x%02x Data: 0x%02x\n", registerAddr, data);
  for (int i = 0; i < NUM_RMS; i++)
  {
    if (handPorts.multiplexer)
    {
      if (selectMultiplexerPort(i))
      {
        printf("\t\tRM on I2C Multiplexer port %d.\n", i);
      }
      else
      {
        printf("\t\tFailed to select port %d.\n", i);
      }
    }

    if ((uint32_t) handPorts.rm[i] == SPI1_BASE)
    {
      setRegisterSPI(handPorts.rm[i], VCNL4010_ADDRESS, registerAddr, data);
      resultOp = writeRegisterSPI(handPorts.rm[i], VCNL4010_ADDRESS, registerAddr);
    }
    else
    {
      setRegisterI2C(handPorts.rm[i], VCNL4010_ADDRESS, registerAddr, data);
      resultOp = writeRegisterI2C(handPorts.rm[i], VCNL4010_ADDRESS, registerAddr);
    }
    
    if (registerAddr != BNO055_SYS_TRIGGER_ADDR) // if not a reset command, check
    {
      if ((uint32_t) handPorts.rm[i] == SPI1_BASE)
      {
        readBytesSPI(handPorts.rm[i], VCNL4010_ADDRESS, 1, response);
      }
      else
      {
        readBytesI2C(handPorts.rm[i], VCNL4010_ADDRESS, 1, response);
      }
      result += (response[0] == data);
    }
    else
    {
      result += resultOp;
    }
  }
  return result == NUM_RMS;
}

uint8_t selectMultiplexerPort(uint8_t port)
{
  if ((uint32_t) handPorts.rm[port] == SPI1_BASE)
  {
    printf("SELECTING RM MULTIPLEXER PORT by SPI WRITE_REGISTER: IMU_NUM %d, PORT %d, ADDR 0x70, REG_ADDR %x",
      port, (int)handPorts.rm[port], 1 << port);
    return writeRegisterSPI(handPorts.rm[port], I2C_MULTIPLEXER_ADDRESS, 1 << port);
  }
  else
  {
    printf("SELECTING RM MULTIPLEXER PORT by I2C WRITE_REGISTER: IMU_NUM %d, PORT %d, ADDR 0x70, REG_ADDR %x",
      port, (int)handPorts.rm[port], 1 << port);
    return writeRegisterI2C(handPorts.rm[port], I2C_MULTIPLEXER_ADDRESS, 1 << port);
  }
}

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