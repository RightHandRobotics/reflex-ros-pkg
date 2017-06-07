#include "rm.h"

// GLOBAL ALL FILES VARIABLE
rm_async_poll_state_t rm_poll_state[NUM_RMS] = {STATE_WAIT, STATE_WAIT, STATE_WAIT}; //silly to have a define when still initialized manually?
uint8_t rm_state_counter[NUM_RMS] = {0, 0, 0}; //var for not getting stuck

/***** USER PARAMETERS *****/
const int ir_current_ = 8;                     // range = [0, 20]. current = value * 10 mA
const int ambient_light_measurement_rate_ = 7; // range = [0, 7]. 1, 2, 3, 4, 5, 6, 8, 10 samples per second
const int averaging_function_ = 7;  // range [0, 7] measurements per run are 2**value, with range [1, 2**7 = 128]
const int proximity_freq_ = 0; // range = [0 , 3]. 390.625kHz, 781.250kHz, 1.5625MHz, 3.125MHz
const int sensitivity = 50;  // Sensitivity of touch/release detection, values closer to zero increase sensitivity

// Touch/release detection
#define EA 0.3  // exponential average weight parameter / cut-off frequency for high-pass filter

void rm_init()
{
  uint8_t id[1] = {0};
  uint8_t result;

  // initializing rms state
  printf("initializing rm state: \n");
  for (int i = 0; i < NUM_RMS; i++)
  {
    result = readRegisterRM(i,PRODUCT_ID,id);
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
  result = setRegisterAllRMs(AMBIENT_PARAMETER, 0x7F);
  printf("\t\tResult: %s\n", result ? "SUCCESS" : "FAILED\n");  
  udelay(1000);

  // set IR current
  printf("\tsetting ir current...\n");
  result = setRegisterAllRMs(IR_CURRENT, ir_current_);
  printf("\t\tResult: %s\n", result ? "SUCCESS" : "FAILED\n");  
  udelay(1000); // takes a while to reset the imus
  
   // set proximity mod
  printf("\tsetting proximity mod...\n");
  result = setRegisterAllRMs(PROXIMITY_MOD, 1);
  printf("\t\tResult: %s\n", result ? "SUCCESS" : "FAILED\n");  
  udelay(1000);
  
  
}

void rm_poll_nonblocking_tick(const uint8_t rmNumber)
{
  static uint16_t proximity_value[NUM_RMS]; // current proximity reading
  static unsigned int average_value[NUM_RMS];   // low-pass filtered proximity reading
  static signed int fa2[NUM_RMS];              // FA-II value;
  static signed int fa2derivative[NUM_RMS];     // Derivative of the FA-II value;
  static signed int fa2deriv_last[NUM_RMS];     // Last value of the derivative (for zero-crossing detection)
  static uint8_t touch[NUM_RMS];
    
  rm_async_poll_state_t* state = (rm_async_poll_state_t*)&(rm_poll_state[rmNumber]);
  uint8_t temp[1] = {0};
  uint8_t result = 0;
  printf("\nRM NUMBER: %d\nSTATE: ", rmNumber);
  switch(*state)
  {
    case RM_STATE_START_MEAS:
      printf("RM_STATE_START_MEAS\n");
      if (rm_state_counter[rmNumber]>3){    // Prevent the hand from getting stuck in a loop when RM connection is lost
        *state = RM_STATE_WAIT;  
      }
      else{
        result = readRegisterRM(rmNumber,COMMAND_0,temp);
        if (result)
           result = setRegisterRM(rmNumber,COMMAND_0,temp[0]|START_PROX_MEAS);
        if (result){
            *state = RM_STATE_READ_PROX;
            rm_state_counter[rmNumber] = 0;
        }
      }
      rm_state_counter[rmNumber]++;
      break;
    case RM_STATE_READ_PROX:
        printf("RM_STATE_READ_PROX\n");
        proximity_value[rmNumber] = 0;
        result = readRegisterRM(rmNumber,COMMAND_0,temp);
        if (result && temp[0] & PROX_MEAS_RDY){
            result = readRegisterRM(rmNumber,PROX_VAL_H,temp); 
            proximity_value[rmNumber] |= temp[0] << 8;
            result &= readRegisterRM(rmNumber,PROX_VAL_H,temp);
            proximity_value[rmNumber] |= temp[0];
            if (result){
                fa2deriv_last[rmNumber] = fa2derivative[rmNumber];
                fa2derivative[rmNumber] = (signed int) average_value[rmNumber] - proximity_value[rmNumber] - fa2[rmNumber];
                fa2[rmNumber] = (signed int) average_value[rmNumber] - proximity_value[rmNumber];
                 if (fa2deriv_last[rmNumber] < -sensitivity && fa2derivative[rmNumber] > sensitivity) {
                    touch[rmNumber] = (fa2[rmNumber]>sensitivity)*1 + (fa2[rmNumber]<-sensitivity)*2; // 
                 }
                average_value[rmNumber] = EA * proximity_value[rmNumber] + (1 - EA) * average_value[rmNumber]; //Do this last
                handState.rm_raw[rmNumber] = proximity_value[rmNumber];
                handState.rm_fa[rmNumber] = fa2[rmNumber];
                handState.rm_touch[rmNumber] = touch[rmNumber];
                *state = RM_STATE_WAIT; 
            }
        } else if (rm_state_counter[rmNumber] > 5){
            *state = IMU_STATE_WAIT;
        }
        rm_state_counter[rmNumber]++;
        break;
    case RM_STATE_WAIT:
      printf("RM_STATE_WAIT\n");
      rm_state_counter[rmNumber] = 0;
      break;
    default:
      printf("UNKNOWN\n");
      *state = IMU_STATE_WAIT;
      break;
  }
}

uint8_t setRegisterAllRMs(uint8_t registerAddr, uint8_t data)
{
  uint8_t result = 0;
  uint8_t resultOp = 0;
  printf("\t\tRegister: 0x%02x Data: 0x%02x\n", registerAddr, data);
  for (int i = 0; i < NUM_RMS; i++)
  {
    if (handPorts.multiplexer)
    {
      if (selectMultiplexerPortRM(i))
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
      resultOp = setRegisterSPI(handPorts.rm[i], VCNL4010_ADDRESS, registerAddr, data);
    }
    else
    {
      resultOp = setRegisterI2C(handPorts.rm[i], VCNL4010_ADDRESS, registerAddr, data);
    }
    result += resultOp;
  }
  return result == NUM_RMS;
}

uint8_t setRegisterRM(int rmNumber, uint8_t registerAddr, uint8_t data)
{
  uint8_t result = 0;
  printf("\t\tRegister: 0x%02x Data: 0x%02x\n", registerAddr, data);

    if (handPorts.multiplexer)
    {
      if (selectMultiplexerPortRM(rmNumber))
      {
        printf("\t\tRM on I2C Multiplexer port %d.\n", rmNumber);
      }
      else
      {
        printf("\t\tFailed to select port %d.\n", rmNumber);
      }
    }

    if ((uint32_t) handPorts.rm[rmNumber] == SPI1_BASE)
    {
      result = setRegisterSPI(handPorts.rm[rmNumber], VCNL4010_ADDRESS, registerAddr, data);
    }
    else
    {
      result = setRegisterI2C(handPorts.rm[rmNumber], VCNL4010_ADDRESS, registerAddr, data);
    }
    
    return result;
}

uint8_t readRegisterRM(int rmNumber, uint8_t registerAddr, uint8_t *data)
{
    uint8_t result = 0;
    if (handPorts.multiplexer) //select multiplexer port
    {
      if (selectMultiplexerPortRM(rmNumber))
      {
          printf("\tI2C Multiplexer port %d, 0x%x: ", rmNumber, 1 << rmNumber);
      }
      else
      {
          printf("\tFailed to select I2C Multiplexer port %d\n ", rmNumber);
      }
    }
    if ((uint32_t) handPorts.rm[rmNumber] == SPI1_BASE)
    {
      result = writeRegisterSPI(handPorts.rm[rmNumber],VCNL4010_ADDRESS, registerAddr);
      result = readBytesSPI(handPorts.rm[rmNumber], VCNL4010_ADDRESS, 1, data);
    }
    else
    {
      result = writeRegisterI2C(handPorts.rm[rmNumber], VCNL4010_ADDRESS, registerAddr);
      result = readBytesI2C(handPorts.rm[rmNumber], VCNL4010_ADDRESS, 1, data);
    }
    return result;
}

uint8_t selectMultiplexerPortRM(uint8_t port)
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
