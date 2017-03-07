#include "takktile.h"

// GLOBAL ALL FILES VARIABLE
takktileAsyncPollState_t takktilePollState[NUM_TACTILE_PORTS] = {STATE_WAIT, STATE_WAIT, STATE_WAIT, STATE_WAIT};

void takktileInit()
{
  printf("takktile sensors initialization...\n");

  // Reset SPI to I2C Converter
  printf("\tresetting SPI to I2C conveter...");
  resetConverter();
  printf(" OK\n");

  // Configure SPI to I2C Conversion
  writeConverterRegister(SC18IS601_REGISTER_I2C_CLOCK, SC18IS601_I2C_CLOCK_369KHZ);
  printf("\t\t I2C Clock Set Value   : %#02x\n", SC18IS601_I2C_CLOCK_369KHZ);
  
  // Print header, all the registers
  printf("\tSPI to I2C Converter registers: \n");
  uint8_t data[1] = {0};
  readConverterRegister(SC18IS601_REGISTER_IO_CONFIG, data);
  printf("\t\t IO Config   : %#02x\n", data[0]);
  readConverterRegister(SC18IS601_REGISTER_IO_STATE, data);
  printf("\t\t IO State    : %#02x\n", data[0]);
  readConverterRegister(SC18IS601_REGISTER_I2C_CLOCK, data);
  printf("\t\t I2C Clock   : %#02x\n", data[0]);
  readConverterRegister(SC18IS601_REGISTER_I2C_TIMEOUT, data);
  printf("\t\t I2C Timeout : %#02x\n", data[0]);
  readConverterRegister(SC18IS601_REGISTER_I2C_STATUS, data);
  printf("\t\t I2C Status  : %#02x\n", data[0]);
  readConverterRegister(SC18IS601_REGISTER_I2C_ADDR, data);
  printf("\t\t I2C Addr    : %#02x\n", data[0]);

  printf("OK\r\n");
  printf("\n");
}

void takktile_poll_nonblocking_tick(const uint8_t takktile_port)
{
  // static uint_fast8_t errCount[NUM_TACTILE_PORTS] = {0};
  static uint8_t sensorNumber[NUM_FINGERS] = {0, 0, 0};
  uint8_t sensorNumberAux;
  uint8_t sensorInMemory;
  uint8_t initialTime[3] = {0}; // for the 3ms delay
  const uint_fast8_t tp = takktile_port; // save typing
  if (tp >= NUM_TACTILE_PORTS)
    return; // let's not corrupt memory.

  takktileAsyncPollState_t *state = &takktilePollState[takktile_port];

  // int *i2c_status = NULL;
  // if (takktile_port == 0 || takktile_port == 1)
  //   i2c_status = (int *)&g_takktile_internal_i2c_status[takktile_port];
  // else if (takktile_port == 2 || takktile_port == 3)
  //   i2c_status = (int *)&g_takktile_bridged_i2c_status[takktile_port-2];
  // else
  //   return; // shouldn't get here... but if somehow we do, it's time to bail

  // if (*i2c_status == TACTILE_I2C_SUCCESS)
  // {
  //   errCount[takktile_port] = 0;
  //   err_unset(ERR_TAC_0_PROBLEM + takktile_port);
  // }
  // else if (*i2c_status == TACTILE_I2C_FAIL)
  // {
  //   if (errCount[takktile_port] > 100)
  //   {
  //     err_set(ERR_TAC_0_PROBLEM + takktile_port);
  //   }
  //   else
  //   {
  //     errCount[takktile_port]++;
  //   }
  // }

  uint8_t takktileNumber;
  if (takktile_port == 2)
    takktileNumber = 1;
  else if (takktile_port == 0)
    takktileNumber = 0;
  else if (takktile_port == 1)
    takktileNumber = 2;
  else
    takktileNumber = 3;
  sensorNumberAux = sensorNumber[takktileNumber];
  sensorInMemory = takktile_port * SENSORS_PER_FINGER + sensorNumberAux;

  // *state = STATE_WAIT;
  // printf("\ntakktile state: ");
  uint8_t result = 0;
  switch (*state)
  {
    case STATE_ENABLE_ALL_SENSORS:
        // printf("STATE_ENABLE_ALL_SENSORS");
        result = enableAllSensors(takktileNumber);
        if (result)
        {
          *state = STATE_START_CONVERSION;
          handStatus.finger[takktileNumber] = 1;
        }
        else
        {
          *state = STATE_START_CONVERSION; //DAVID
          handStatus.finger[takktileNumber] = 0;
        }
      break;
    case STATE_START_CONVERSION:
        // printf("STATE_START_CONVERSION");
        // if (startConversionSequence(takktileNumber))
        // {
        //   *state = STATE_DISABLE_ALL_SENSORS;
        //   initialTime[takktileNumber] = SYSTIME; // start the timer
        // }
        startConversionSequence(takktileNumber); //DAVID
        *state = STATE_DISABLE_ALL_SENSORS; //DAVID
        initialTime[takktileNumber] = SYSTIME; //DAVID
      break;
    case STATE_DISABLE_ALL_SENSORS:
        // printf("STATE_DISABLE_ALL_SENSORS");
        if (disableAllSensors(takktileNumber))
        {
          *state = STATE_ENABLE_SENSOR;
          handStatus.finger[takktileNumber] = 1;
        }
        else
        {
          *state = STATE_ENABLE_SENSOR; //DAVID
          handStatus.finger[takktileNumber] = 0; 
        }
      break;
    case STATE_ENABLE_SENSOR:
        // udelay(3000);
        // printf("STATE_ENABLE_SENSOR");
        if(sensorNumberAux != 0 || (SYSTIME - initialTime[takktileNumber] > 3000))
        {
          if (enableSensor(takktileNumber, sensorNumberAux))
          {
            *state = STATE_SET_REGISTER;
            handStatus.finger[takktileNumber] = 1;
          }
          else
          {
            *state = STATE_SET_REGISTER; //DAVID
            handStatus.finger[takktileNumber] = 0; 
          }
        }
      break;
    case STATE_SET_REGISTER:
        // printf("STATE_SET_REGISTER");
        if (setRegister(takktileNumber))
        {
          *state = STATE_READ_VALUES;
          handStatus.takktileSensor[sensorInMemory] = 1;
        }
        else
        {
          *state = STATE_READ_VALUES; //DAVID
          handStatus.takktileSensor[sensorInMemory] = 0; 
        }
      break;
    case STATE_READ_VALUES:
        // printf("STATE_READ_VALUES");
        if (readValues(takktileNumber, sensorNumberAux))
        {
          *state = STATE_DISABLE_SENSOR;
          handStatus.takktileSensor[sensorInMemory] = 1;
        }
        else
        {
          *state = STATE_DISABLE_SENSOR; //DAVID
          handStatus.takktileSensor[sensorInMemory] = 0; 
        }
      break;
    case STATE_DISABLE_SENSOR:
        // printf("STATE_DISABLE_SENSOR");
        result = disableSensor(takktileNumber, sensorNumberAux);
        sensorNumber[takktileNumber]++;
        if (sensorNumberAux + 2 > SENSORS_PER_FINGER)
        {
          // printf("\nX");
          *state = STATE_WAIT;
          sensorNumber[takktileNumber] = 0;
          handStatus.finger[takktileNumber] = 1;
        }
        else if (result)
        {
          // printf("\nY");
          *state = STATE_ENABLE_SENSOR;
          handStatus.finger[takktileNumber] = 1;
        }
        else
        {
          // printf("\nZ");
          handStatus.finger[takktileNumber] = 0;
        }
      break;
    case STATE_WAIT:
      // printf("STATE_WAIT"); 
      break;
    default:
      // printf("UNKNOWN");
      *state = STATE_WAIT;
      break;
  }
}

uint8_t enableAllSensors(uint8_t takktileNumber)
{
  uint8_t result = 0;
  printf("ENABLING ALL SENSORS by I2C/SPI WRITE: takktileNumber %d, ADDR 0x0C, data NULL, length 0", takktileNumber);
  switch ((uint32_t) handPorts.takktile[takktileNumber])
  {
    case I2C1_BASE:
      result = writeBytesI2C(handPorts.takktile[takktileNumber], BCAST_ENABLE_ADDR, NULL, 0, 1);  // enable all sensors  
    break;
    case I2C3_BASE:
      result = writeBytesI2C(handPorts.takktile[takktileNumber], BCAST_ENABLE_ADDR, NULL, 0, 1);  // enable all sensors  
    break;
    case SPI1_BASE:
      result = writeBytesSPI(handPorts.takktile[takktileNumber], BCAST_ENABLE_ADDR, NULL, 0, 1);  // enable all sensors  
    break;
  }
  return result;
}

uint8_t startConversionSequence(uint8_t takktileNumber)
{
  uint8_t result = 0;
  uint8_t data[2] = {0x12, 0x01};
  printf("STARTING CONVERSION SEQUENCE by I2C/SPI WRITE: takktileNumber %d, ADDR 0xC0, data {0x12, 0x01}, length 2", takktileNumber);
  switch ((uint32_t) handPorts.takktile[takktileNumber])
  {
    case I2C1_BASE:
      result = writeBytesI2C(handPorts.takktile[takktileNumber], BAROM_ADDR, data, 2, 1);         // send Start Conversion Sequence
    break;
    case I2C3_BASE:
      result = writeBytesI2C(handPorts.takktile[takktileNumber], BAROM_ADDR, data, 2, 1);         // send Start Conversion Sequence
    break;
    case SPI1_BASE:
      result = writeBytesSPI(handPorts.takktile[takktileNumber], BAROM_ADDR, data, 2, 1);         // send Start Conversion Sequence
    break;
  }
  return result;
}

uint8_t disableAllSensors(uint8_t takktileNumber)
{
  uint8_t result = 0;
  printf("DISABLING ALL SENSORS by I2C/SPI READ: takktileNumber %d, ADDR 0x0D>>1, length 1", takktileNumber);
  switch ((uint32_t) handPorts.takktile[takktileNumber])
  {
    uint8_t aux[1] = {0};
    case I2C1_BASE:
      result = readBytesI2C(handPorts.takktile[takktileNumber], BCAST_DISABLE_ADDR >> 1, 1, aux); // disable all sensors
    break;
    case I2C3_BASE:
      result = readBytesI2C(handPorts.takktile[takktileNumber], BCAST_DISABLE_ADDR >> 1, 1, aux); // disable all sensors
    break;
    case SPI1_BASE:
      result = readBytesSPI(handPorts.takktile[takktileNumber], BCAST_DISABLE_ADDR >> 1, 1, aux); // disable all sensors
    break;
  }  
  return result;
}

uint8_t enableSensor(uint8_t takktileNumber, uint8_t sensorIndex)
{
  uint8_t result = 0;
  uint8_t addresses[9] = {0x00, 0x02, 0x04, 0x06, 0x08, 0x10, 0x12, 0x14, 0x16};
  printf("ENABLING SENSOR by I2C/SPI WRITE: takktileNumber %d, ADDR %x, data NULL, length 0", takktileNumber, addresses[sensorIndex]);
  switch ((uint32_t) handPorts.takktile[takktileNumber])
  {
    case I2C1_BASE:
      result = writeBytesI2C(handPorts.takktile[takktileNumber], addresses[sensorIndex], NULL, 0, 1); // enable sensor i
    break;
    case I2C3_BASE:
      result = writeBytesI2C(handPorts.takktile[takktileNumber], addresses[sensorIndex], NULL, 0, 1); // enable sensor i
    break;
    case SPI1_BASE:
      result = writeBytesSPI(handPorts.takktile[takktileNumber], addresses[sensorIndex], NULL, 0, 1); // enable sensor i
    break;
  }
  return result;
}

uint8_t setRegister(uint8_t takktileNumber)
{
  uint8_t result = 0;
  uint8_t msg[1] = {0x00};
  printf("SETTING REGISTER by I2C/SPI WRITE: takktileNumber %d, ADDR 0xC0, data {0x00}, length 1", takktileNumber);
  switch ((uint32_t) handPorts.takktile[takktileNumber])
  {
    case I2C1_BASE:
      result = writeBytesI2C(handPorts.takktile[takktileNumber], BAROM_ADDR, msg, 1, 1);       // choose register 0x00
    break;
    case I2C3_BASE:
      result = writeBytesI2C(handPorts.takktile[takktileNumber], BAROM_ADDR, msg, 1, 1);       // choose register 0x00
    break;
    case SPI1_BASE:
      result = writeBytesSPI(handPorts.takktile[takktileNumber], BAROM_ADDR, msg, 1, 1);       // choose register 0x00
    break;
  }
  return result;
}

uint8_t readValues(uint8_t takktileNumber, uint8_t sensorIndex)
{
  uint8_t result = 0;
  uint8_t tp;
  uint8_t index;
  uint8_t values[4] = {0, 0, 0, 0};
  printf("READING VALUES by I2C/SPI READ: takktileNumber %d, ADDR 0xC0>>1, length 4", takktileNumber);
  switch ((uint32_t) handPorts.takktile[takktileNumber])
  {
    case I2C1_BASE:
      result = readBytesI2C(handPorts.takktile[takktileNumber], BAROM_ADDR >> 1, 4, values);      // read 4 bytes
    break;
    case I2C3_BASE:
      result = readBytesI2C(handPorts.takktile[takktileNumber], BAROM_ADDR >> 1, 4, values);      // read 4 bytes
    break;
    case SPI1_BASE:
      result = readBytesSPI(handPorts.takktile[takktileNumber], BAROM_ADDR >> 1, 4, values);      // read 4 bytes
    break;
  }
  if (takktileNumber == 2)
    tp = 1;
  else if (takktileNumber == 0)
    tp = 0;
  else if (takktileNumber == 1)
    tp = 2;
  else
    tp = 3;
  index = tp * SENSORS_PER_FINGER + sensorIndex;
  handState.takktile_pressures[index] = 510 - (values[0]<200 ? ((uint16_t)values[0] + 255) : ((uint16_t)values[0]));
  handState.takktile_temperatures[index] = ((uint16_t)values[2] << 2) | (values[3] >> 6);
  // printf("%3d ", handState.takktile_pressures[index]);
  udelay(SLEEP_TIME);
  return result;
}

uint8_t disableSensor(uint8_t takktileNumber, uint8_t sensorIndex)
{
  uint8_t result = 0;
  uint8_t addresses[9] = {0x00, 0x02, 0x04, 0x06, 0x08, 0x10, 0x12, 0x14, 0x16};
  uint8_t aux[1] = {0};
  printf("DISABLING SENSOR by I2C/SPI READ: takktileNumber %d, ADDR %x, length 1", takktileNumber, addresses[sensorIndex] >> 1);
  switch ((uint32_t) handPorts.takktile[takktileNumber])
  {
    case I2C1_BASE:
      result = readBytesI2C(handPorts.takktile[takktileNumber], addresses[sensorIndex] >> 1, 1, aux);
    break;
    case I2C3_BASE:
      result = readBytesI2C(handPorts.takktile[takktileNumber], addresses[sensorIndex] >> 1, 1, aux);
    break;
    case SPI1_BASE:
      result = readBytesSPI(handPorts.takktile[takktileNumber], addresses[sensorIndex] >> 1, 1, aux);
    break;
  }
  return result;
}






