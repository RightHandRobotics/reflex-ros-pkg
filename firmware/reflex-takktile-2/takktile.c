#include "takktile.h"

// GLOBAL ALL FILES VARIABLE
takktileAsyncPollState_t takktilePollState[NUM_TACTILE_PORTS] = {STATE_WAIT, STATE_WAIT, STATE_WAIT, STATE_WAIT};

void takktileInit()  //Initialize the takktile sensors for use
{
  printf("takktile sensors initialization...\n");

  // // Reset SPI to I2C Converter
  // printf("\tresetting SPI to I2C conveter...");
  // resetConverter();
  // printf(" OK\n");

  // // Configure SPI to I2C Conversion
  // writeConverterRegister(SC18IS601_REGISTER_I2C_CLOCK, SC18IS601_I2C_CLOCK_369KHZ);
  
  // // Print header, all the registers
  // // printf("\tSPI to I2C Converter registers: \n");
  // // uint8_t data[1] = {0};
  // // readConverterRegister(SC18IS601_REGISTER_IO_CONFIG, data);
  // // printf("\t\t IO Config   : %#02x\n", data[0]);
  // // readConverterRegister(SC18IS601_REGISTER_IO_STATE, data);
  // // printf("\t\t IO State    : %#02x\n", data[0]);
  // // readConverterRegister(SC18IS601_REGISTER_I2C_CLOCK, data);
  // // printf("\t\t I2C Clock   : %#02x\n", data[0]);
  // // readConverterRegister(SC18IS601_REGISTER_I2C_TIMEOUT, data);
  // // printf("\t\t I2C Timeout : %#02x\n", data[0]);
  // // readConverterRegister(SC18IS601_REGISTER_I2C_STATUS, data);
  // // printf("\t\t I2C Status  : %#02x\n", data[0]);
  // // readConverterRegister(SC18IS601_REGISTER_I2C_ADDR, data);
  // // printf("\t\t I2C Addr    : %#02x\n", data[0]);

  // // printf("OK\r\n");
  // // printf("\n");
}

//Update finger or takktile status values in the handStatus struct
void updateFingerStatus(uint8_t fingerNumber, uint8_t status){
  if (fingerNumber >= NUM_FINGERS) //Don't corrupt memory
    return;

  handStatus.takktileFinger[fingerNumber].fingerStatus = status;
}

void updateTakktileStatus(uint8_t fingerNumber, uint8_t takktileNumber, uint8_t status){
  if (takktileNumber <= SENSORS_PER_FINGER){
    handStatus.takktileFinger[fingerNumber].takktileSensor[takktileNumber] = status;
  }
}

//Check finger or takktile status values in the handStatus struct
uint8_t checkFingerStatus(uint8_t fingerNumber){
  if (fingerNumber >= NUM_FINGERS) //Don't corrupt memory
    return 0;
  
  return handStatus.takktileFinger[fingerNumber].fingerStatus;
}

uint8_t checkTakktileStatus(uint8_t fingerNumber, uint8_t takktileNumber){
  if (takktileNumber <= SENSORS_PER_FINGER){
    return handStatus.takktileFinger[fingerNumber].takktileSensor[takktileNumber];
  }
  return 0;
}

void takktile_poll_nonblocking_tick(const uint8_t takktileNumber)
{
  //Function to handle the takktile polling state machine
  static uint8_t sensorNumber[NUM_FINGERS] = {0, 0, 0};
  static uint_fast8_t sumStatus[NUM_FINGERS] = {0, 0, 0};
  uint8_t sensorNumberAux;
  if (takktileNumber >= NUM_FINGERS)
    return; // let's not corrupt memory.

  takktileAsyncPollState_t *state = &takktilePollState[takktileNumber];
  
  sensorNumberAux = sensorNumber[takktileNumber];  //ID number of the takktile sensor on the finger

  if (checkFingerStatus(takktileNumber) == 1){  //Only poll if the finger is working
    uint8_t result = 0;
    switch (*state)
    {
      case STATE_ENABLE_ALL_SENSORS:
          if (!enableAllSensors(takktileNumber))
          {
            updateFingerStatus(takktileNumber, 0);//handStatus.finger[takktileNumber] = 0;
          }
          *state = STATE_START_CONVERSION;
        break;
      case STATE_START_CONVERSION:
          startConversionSequence(takktileNumber); //DAVID
          *state = STATE_DISABLE_ALL_SENSORS; //DAVID
          //initialTime = SYSTIME; //DAVID
        break;
      case STATE_DISABLE_ALL_SENSORS:
          if (!disableAllSensors(takktileNumber))
          {
            updateFingerStatus(takktileNumber, 0);//handStatus.finger[takktileNumber] = 0; 
          }
          *state = STATE_ENABLE_SENSOR;
        break;
      case STATE_ENABLE_SENSOR:
        //if (SYSTIME - initialTime > 3000){
          if((checkTakktileStatus(takktileNumber, sensorNumberAux)==1))// || (SYSTIME - initialTime[takktileNumber] > 3000))
          {
            if (enableSensor(takktileNumber, sensorNumberAux))
            {
              *state = STATE_SET_REGISTER;
            }
            else
            {
              *state = STATE_DISABLE_SENSOR; //DAVID
              updateTakktileStatus(takktileNumber, sensorNumberAux, 0);//handStatus.takktileSensor[sensorInMemory] = 0; 
            }
          }
          else{
            *state = STATE_DISABLE_SENSOR;
          }
        //}
        break;
      case STATE_SET_REGISTER:
          //if (SYSTIME - initialTime > 3000){
            result = setRegister(takktileNumber);
            if (result)
            {
              *state = STATE_READ_VALUES;
            }
            else
            {
              *state = STATE_DISABLE_SENSOR; //DAVID
              updateTakktileStatus(takktileNumber, sensorNumberAux, 0);//handStatus.takktileSensor[sensorInMemory] = 0; 
            }
          //}
          // printf("Setting reg");NUM_QUATERNIONS
        break;
      case STATE_READ_VALUES:
          if (sensorNumberAux == 0)
            readValues(takktileNumber, sensorNumberAux);// This is such a huge hack
          else
            readValues(takktileNumber, sensorNumberAux - 1); // This is such a huge hack
          sumStatus[takktileNumber]++;
          *state = STATE_DISABLE_SENSOR;
           // printf("f: %d, s: %d\n", takktileNumber, sensorNumberAux);
        break;
      case STATE_DISABLE_SENSOR:
          if (checkTakktileStatus(takktileNumber, sensorNumberAux) == 1)
            result = disableSensor(takktileNumber, sensorNumberAux);
          sensorNumber[takktileNumber]+=1;
          // printf("s[]: %d\n", sensorNumber[takktileNumber]);
          if (sensorNumberAux + 2 > (SENSORS_PER_FINGER+1)) // 14 // (?)// This is such a huge hack
          {
            *state = STATE_WAIT;
            sensorNumber[takktileNumber] = 0;
            if (sumStatus[takktileNumber] < 2)
              updateFingerStatus(takktileNumber, 0);
            else
              sumStatus[takktileNumber] = 0;
          }
          else if (result)
          {
            *state = STATE_ENABLE_SENSOR;
            updateTakktileStatus(takktileNumber, sensorNumberAux, 1);//handStatus.finger[takktileNumber] = 1;
          }
          else
          {
            *state = STATE_ENABLE_SENSOR;
            updateTakktileStatus(takktileNumber, sensorNumberAux, 0);//handStatus.finger[takktileNumber] = 0;
          }

          // if ((*state == STATE_ENABLE_SENSOR) && (checkTakktileStatus(takktileNumber, sensorNumberAux + 1) == 0)){
          //   *state = STATE_DISABLE_SENSOR;
          // }
        break;
      case STATE_WAIT:
        break;
      default:
        *state = STATE_WAIT;
        break;
    }
  }
  else
    *state = STATE_WAIT;
}

uint8_t enableAllSensors(uint8_t takktileNumber)
{
  uint8_t result = 0;
  //printf("ENABLING ALL SENSORS by I2C/SPI WRITE: takktileNumber %d, ADDR 0x0C, data NULL, length 0\n", takktileNumber);
  switch ((uint32_t) handPorts.takktile[takktileNumber])
  {
    case I2C1_BASE:
      result = writeBytesI2C(handPorts.takktile[takktileNumber], BCAST_ENABLE_ADDR, NULL, 0, 1);  // enable all sensors
	  if (0 == result){
		  resetI2C(handPorts.takktile[takktileNumber]);
	  }
    break;
    case I2C3_BASE:
      result = writeBytesI2C(handPorts.takktile[takktileNumber], BCAST_ENABLE_ADDR, NULL, 0, 1);  // enable all sensors
	  if (0 == result){
		  resetI2C(handPorts.takktile[takktileNumber]);
	  }
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
  //printf("STARTING CONVERSION SEQUENCE by I2C/SPI WRITE: takktileNumber %d, ADDR 0xC0, data {0x12, 0x01}, length 2\n", takktileNumber);
  switch ((uint32_t) handPorts.takktile[takktileNumber])
  {
    case I2C1_BASE:
      result = writeBytesI2C(handPorts.takktile[takktileNumber], BAROM_ADDR, data, 2, 1);         // send Start Conversion Sequence
	  if (0 == result){
		  resetI2C(handPorts.takktile[takktileNumber]);
	  }
    break;
    case I2C3_BASE:
      result = writeBytesI2C(handPorts.takktile[takktileNumber], BAROM_ADDR, data, 2, 1);         // send Start Conversion Sequence
	  if (0 == result){
		  resetI2C(handPorts.takktile[takktileNumber]);
	  }
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
  //printf("DISABLING ALL SENSORS by I2C/SPI READ: takktileNumber %d, ADDR 0x0D>>1, length 1\n", takktileNumber);
  switch ((uint32_t) handPorts.takktile[takktileNumber])
  {
    uint8_t aux[1] = {0};
    case I2C1_BASE:
      result = readBytesI2C(handPorts.takktile[takktileNumber], BCAST_DISABLE_ADDR >> 1, 1, aux); // disable all sensors
	  if (0 == result){
		  resetI2C(handPorts.takktile[takktileNumber]);
	  }
    break;
    case I2C3_BASE:
      result = readBytesI2C(handPorts.takktile[takktileNumber], BCAST_DISABLE_ADDR >> 1, 1, aux); // disable all sensors
	  if (0 == result){
		  resetI2C(handPorts.takktile[takktileNumber]);
	  }
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
  //uint8_t addresses[14] = {0x00, 0x02, 0x04, 0x06, 0x08, 0x10, 0x12, 0x14, 0x16, 0x18, 0x20, 0x22, 0x24, 0x26};
  uint8_t addresses[15] = {0x28, 0x00, 0x02, 0x04, 0x06, 0x08, 0x10, 0x12, 0x14, 0x16, 0x18, 0x20, 0x22, 0x24, 0x26};// This is such a huge hack
  //printf("ENABLING SENSOR by I2C/SPI WRITE: takktileNumber %d, ADDR %x, data NULL, length 0\n", takktileNumber, addresses[sensorIndex]);
  switch ((uint32_t) handPorts.takktile[takktileNumber])
  {
    case I2C1_BASE:
      result = writeBytesI2C(handPorts.takktile[takktileNumber], addresses[sensorIndex], NULL, 0, 1); // enable sensor i
	  if (0 == result){
		  resetI2C(handPorts.takktile[takktileNumber]);
	  }
    break;
    case I2C3_BASE:
      result = writeBytesI2C(handPorts.takktile[takktileNumber], addresses[sensorIndex], NULL, 0, 1); // enable sensor i
	  if (0 == result){
		  resetI2C(handPorts.takktile[takktileNumber]);
	  }
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
  //printf("SETTING REGISTER by I2C/SPI WRITE: takktileNumber %d, ADDR 0xC0, data {0x00}, length 1\n", takktileNumber);
  switch ((uint32_t) handPorts.takktile[takktileNumber])
  {
    case I2C1_BASE:
      result = writeBytesI2C(handPorts.takktile[takktileNumber], BAROM_ADDR, msg, 1, 1);       // choose register 0x00
	  if (0 == result){
		  resetI2C(handPorts.takktile[takktileNumber]);
	  }
    break;
    case I2C3_BASE:
      result = writeBytesI2C(handPorts.takktile[takktileNumber], BAROM_ADDR, msg, 1, 1);       // choose register 0x00
	  if (0 == result){
		  resetI2C(handPorts.takktile[takktileNumber]);
	  }
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
  //printf("READING VALUES by I2C/SPI READ: takktileNumber %d, ADDR 0xC0>>1, length 4\n", takktileNumber);
  switch ((uint32_t) handPorts.takktile[takktileNumber])
  {
    case I2C1_BASE:
      result = readBytesI2C(handPorts.takktile[takktileNumber], BAROM_ADDR >> 1, 4, values);      // read 4 bytes
	  if (0 == result){
		  resetI2C(handPorts.takktile[takktileNumber]);
	  }
    break;
    case I2C3_BASE:
      result = readBytesI2C(handPorts.takktile[takktileNumber], BAROM_ADDR >> 1, 4, values);      // read 4 bytes
	  if (0 == result){
		  resetI2C(handPorts.takktile[takktileNumber]);
	  }
    break;
    case SPI1_BASE:
      result = readBytesSPI(handPorts.takktile[takktileNumber], BAROM_ADDR >> 1, 4, values);      // read 4 bytes
    break;
  }
  if (takktileNumber == 2)
    tp = 1; // shouldn't this be 2?
  else if (takktileNumber == 0)
    tp = 0;
  else if (takktileNumber == 1)
    tp = 2; // shouldn't this be 1?
  else {
    printf("got to the weird place\n"); //should never get here
    tp = 3;
  }

  index = tp * SENSORS_PER_FINGER + sensorIndex; // - 1
  
  handState.takktile_pressures[index] = 510 - (values[0]<200 ? ((uint16_t)values[0] + 255) : ((uint16_t)values[0]));
  handState.takktile_temperatures[index] = ((uint16_t)values[2] << 2) | (values[3] >> 6);
  //udelay(SLEEP_TIME);
  return result;
}

uint8_t disableSensor(uint8_t takktileNumber, uint8_t sensorIndex)
{
  uint8_t result = 0;
  //uint8_t addresses[14] = {0x00, 0x02, 0x04, 0x06, 0x08, 0x10, 0x12, 0x14, 0x16, 0x18, 0x20, 0x22, 0x24, 0x26};
  uint8_t addresses[15] = {0x28, 0x00, 0x02, 0x04, 0x06, 0x08, 0x10, 0x12, 0x14, 0x16, 0x18, 0x20, 0x22, 0x24, 0x26};// This is such a huge hack
  uint8_t aux[1] = {0};
  //printf("DISABLING SENSOR by I2C/SPI READ: takktileNumber %d, ADDR %x, length 1\n", takktileNumber, addresses[sensorIndex] >> 1);
  switch ((uint32_t) handPorts.takktile[takktileNumber])
  {
    case I2C1_BASE:
      result = readBytesI2C(handPorts.takktile[takktileNumber], addresses[sensorIndex] >> 1, 1, aux);
	  if (0 == result){
		  resetI2C(handPorts.takktile[takktileNumber]);
	  }
    break;
    case I2C3_BASE:
      result = readBytesI2C(handPorts.takktile[takktileNumber], addresses[sensorIndex] >> 1, 1, aux);
	  if (0 == result){
		  resetI2C(handPorts.takktile[takktileNumber]);
	  }
    break;
    case SPI1_BASE:
      result = readBytesSPI(handPorts.takktile[takktileNumber], addresses[sensorIndex] >> 1, 1, aux);
    break;
  }
  return result;
}
