#include "imu.h"

// Global variables
// Set default states to STATE_WAIT for each IMU upon power-on
imu_async_poll_state_t imu_poll_state[NUM_IMUS] = {IMU_STATE_WAIT, IMU_STATE_WAIT, IMU_STATE_WAIT, IMU_STATE_WAIT};

// Set default poll type to IMU_DATA for each IMU upon power-on
imu_poll_type_t imu_poll_type[NUM_IMUS] = {IMU_DATA, IMU_DATA, IMU_DATA, IMU_DATA};

// Byte-sized array for counting how "long" an IMU stays in one state
uint8_t imu_state_count[NUM_IMUS] = {0, 0, 0, 0};

// Flag that is set in IMU_CAL_OFFSETS. Indicates if the calibration values are read. Never set back to zero.
uint8_t imu_cal_values_read[NUM_QUATERNIONS] = {0, 0, 0, 0};
uint8_t imu_cal_status_count[NUM_IMUS] = {0, 0, 0, 0};

// Flag indiciates if calibration values are set.
uint8_t imu_cal_values_set[NUM_QUATERNIONS] = {0, 0, 0, 0};


/*
  Description: Initializes and sets important stuff. called in main.c
  Returns: uint8_t result
*/
void imuInit()
{
  uint8_t id[1] = {0}; // This initializes the variable in one line. {} is for a pointer. No need to make memory and stuff
  
  uint8_t result;

  printf("Initializing IMU state: \n");

  // For each IMU
  for (int i = 0; i < NUM_IMUS; i++) 
  {
    if (handPorts.multiplexer) // multiplexer is an attribute of ports_t struct
    {
      if (selectMultiplexerPort(i))
          printf("\tI2C Multiplexer port %d, 0x%x: ", i, 1 << i);
      else
          printf("\tFailed to select I2C Multiplexer port %d\n ", i);
    }

    // SPI
    if ((uint32_t) handPorts.imu[i] == SPI1_BASE)
    {
      result = writeRegisterSPI(handPorts.imu[i], handPorts.imuI2CAddress[i], BNO055_CHIP_ID_ADDR);
	if (0 == result){
		converterInit();
	}
      result = readBytesSPI(handPorts.imu[i], handPorts.imuI2CAddress[i], 1, id);
	if (0 == result){
		converterInit();
	}
    }

    // I2C
    else
    {
      result = writeRegisterI2C(handPorts.imu[i], handPorts.imuI2CAddress[i], BNO055_CHIP_ID_ADDR);
	  if (0 == result){
		  resetI2C(handPorts.imu[i]);
	  }
      result = readBytesI2C(handPorts.imu[i], handPorts.imuI2CAddress[i], 1, id);
	  if (0 == result){
		  resetI2C(handPorts.imu[i]);
	  }
    }

    // Check if ID is correct
    if (*id != BNO055_ID){
      udelay(1000); // Hold on for boot
      if ((uint32_t) handPorts.imu[i] == SPI1_BASE)
      {
        result = writeRegisterSPI(handPorts.imu[i], handPorts.imuI2CAddress[i], BNO055_CHIP_ID_ADDR);
	if (0 == result){
		converterInit();
	}
        result = readBytesSPI(handPorts.imu[i], handPorts.imuI2CAddress[i], 1, id);
	if (0 == result){
		converterInit();
	}
      }

      // I2C
      else
      {
        result = writeRegisterI2C(handPorts.imu[i], handPorts.imuI2CAddress[i], BNO055_CHIP_ID_ADDR);
		  if (0 == result){
			  resetI2C(handPorts.imu[i]);
		  }
        result = readBytesI2C(handPorts.imu[i], handPorts.imuI2CAddress[i], 1, id);
		  if (0 == result){
			  resetI2C(handPorts.imu[i]);
		  }
      }
      if (*id != BNO055_ID)
        printf("IMU %d not found. ID: %d, Address: 0x%x Result: %d\n", i, id[0], handPorts.imuI2CAddress[i], result);
      else 
        printf("IMU %d found. ID: %d, Address: 0x%x Result: %d\n", i, id[0], handPorts.imuI2CAddress[i], result);
    }
    else
      printf("IMU %d found. ID: %d, Address: 0x%x Result: %d\n", i, id[0], handPorts.imuI2CAddress[i], result);
  }

////////////////////////////////////////////////////////////////////////////////////////
// Understand the OPERATION MODES and transitions
// Operation modes are found in imu.h line 202 bno055_opmode_t

  // Set operation mode to CONFIG_MODE
  printf("\tSetting operation mode...\n");
  result = setRegisterIMUs(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG); 
  printf("\t\tResult: %s\n", result ? "SUCCESS" : "FAILED\n");  
  udelay(19000);

  // Reset system. Set RST_SYS bit in SYS_TRIGGER register
  printf("\tResetting...\n");
  result = setRegisterIMUs(BNO055_SYS_TRIGGER_ADDR, 0x20);
  printf("\t\tResult: %s\n", result ? "SUCCESS" : "FAILED\n");  
  udelay(1000000); // Takes a while to reset the IMUs
  
   // Set power mode to normal
  printf("\tSetting power modes...\n");
  result = setRegisterIMUs(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
  printf("\t\tResult: %s\n", result ? "SUCCESS" : "FAILED\n");  
  udelay(19000);
  
  // Set page ID. Change page to 0x00. Most registers needed are on page 0x00.
  printf("\tSetting page ID...\n");
  result = setRegisterIMUs(BNO055_PAGE_ID_ADDR, 0);
  printf("\t\tResult: %s\n", result ? "SUCCESS" : "FAILED\n");  
  udelay(19000);
  
  // Set external crystal use ID
  printf("\tSetting external crystal use ID...\n");
  result = setRegisterIMUs(BNO055_SYS_TRIGGER_ADDR, 0x80);
  printf("\t\tResult: %s\n", result ? "SUCCESS" : "FAILED\n");  
  udelay(1000000); //500000); // Takes a while to set this configuration
    
  // Set operating mode to NDOF (nine degree of freedom) FUSION
  printf("\tSetting operation mode 0x%02x... \n", OPERATION_MODE_NDOF);
  result = setRegisterIMUs(BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF);
  printf("\t\tResult: %s\n", result ? "SUCCESS" : "FAILED\n");
  udelay(19000);
  
}

/*
void enterConfigMode(bool result){
  result = setRegisterIMUs(BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF);
}
*/


////////////////////////////////////////////////////////////////////////////////////////

/*
  Description: Sets (write to) IMU registers
  Returns: result: 1 if SUCCESS, 0 if FAILED to set
*/
uint8_t setRegisterIMUs(uint8_t registerAddr, uint8_t data)
{
  printf("\t\tRegister: 0x%02x Data: 0x%02x\n", registerAddr, data);

  // Set specified register to hold the given byte of data
  uint8_t result = 0;
  uint8_t res = 0;
  uint8_t resultOp = 0;       // Used for writeRegisterSPI
  uint8_t response[1] = {0}; 

  // Loop through each IMU
  for (int i = 0; i < NUM_IMUS; i++)
  {
    if (handPorts.multiplexer)
    {
      if (selectMultiplexerPort(i))
        printf("\t\tIMU on I2C Multiplexer port %d.\n", i);
      else
        printf("\t\tFailed to select port %d.\n", i);
    }

    // Send data to IMU register using I2C or SPI-to-I2C bridge (depends on finger)
    if ((uint32_t) handPorts.imu[i] == SPI1_BASE)
    {
      resultOp = setRegisterSPI(handPorts.imu[i], handPorts.imuI2CAddress[i], registerAddr, data);
	if (0 == resultOp){
		converterInit();
	}
      //resultOp = writeRegisterSPI(handPorts.imu[i], handPorts.imuI2CAddress[i], registerAddr);
    }

    else
    {
      resultOp = setRegisterI2C(handPorts.imu[i], handPorts.imuI2CAddress[i], registerAddr, data);
	  if (0 == resultOp){
		  resetI2C(handPorts.imu[i]);
	  }
     //resultOp = writeRegisterI2C(handPorts.imu[i], handPorts.imuI2CAddress[i], registerAddr);
    }
    
    // Check to make sure data is sent correctly
    if (registerAddr != BNO055_SYS_TRIGGER_ADDR) // If not a reset command, check
    {
      if ((uint32_t) handPorts.imu[i] == SPI1_BASE){
        res = readBytesSPI(handPorts.imu[i], handPorts.imuI2CAddress[i], 1, response);
	if (0 == res){
		converterInit();
	}
      }
      else {
        res = readBytesI2C(handPorts.imu[i], handPorts.imuI2CAddress[i], 1, response);
		  if (0 == res){
			  resetI2C(handPorts.imu[i]);
		  }
      }
      result += (response[0] == data);
    }

    else
      result += resultOp;
    
  }

  return result;
  //return result == NUM_IMUS;
}

/*
  Description: Sets IMU registers
  Returns: result: 1 if SUCCESS, 0 if FAILED to set. TODO(LANCE): change return type to bool
*/
uint8_t setRegisterIMU(uint8_t num, uint8_t registerAddr, uint8_t data)
{
  // printf("\t\tRegister: 0x%02x Data: 0x%02x\n", registerAddr, data);

  // Set specified register to hold given byte of data
  uint8_t result = 0;
  uint8_t resultOp = 0;
  uint8_t response[1] = {0};
  uint8_t res = 0;

  if (handPorts.multiplexer)
  {
    if (selectMultiplexerPort(num))
      printf("\t\tIMU on I2C Multiplexer port %d.\n", num);
    else
      printf("\t\tFailed to select port %d.\n", num);
  }

  // Send data to IMU register using I2C or SPI-to-I2C bridge (depends on finger)
  if ((uint32_t) handPorts.imu[num] == SPI1_BASE)
  {
    resultOp = setRegisterSPI(handPorts.imu[num], handPorts.imuI2CAddress[num], registerAddr, data);
	if (0 == resultOp){
		converterInit();
	}
   //resultOp = writeRegisterSPI(handPorts.imu[num], handPorts.imuI2CAddress[num], registerAddr);
  }

  else
  {
    resultOp = setRegisterI2C(handPorts.imu[num], handPorts.imuI2CAddress[num], registerAddr, data);
	  if (0 == resultOp){
		  resetI2C(handPorts.imu[num]);
	  }
    //resultOp = writeRegisterI2C(handPorts.imu[num], handPorts.imuI2CAddress[num], registerAddr);
  }
  
  // Check to make sure data is sent correctly
  if (registerAddr != BNO055_SYS_TRIGGER_ADDR) // If not a reset command, check
  {
    if ((uint32_t) handPorts.imu[num] == SPI1_BASE){
      res = readBytesSPI(handPorts.imu[num], handPorts.imuI2CAddress[num], 1, response);
	if (0 == res){
		converterInit();
	}
    }
    else {
      res = readBytesI2C(handPorts.imu[num], handPorts.imuI2CAddress[num], 1, response);
		if (0 == res){
			resetI2C(handPorts.imu[num]);
		}
    }
    result += (response[0] == data);
  }

  else
    result += resultOp;

  return result;
  //return result == NUM_IMUS;
}
/*
  Description: Selects the IMU multiplexer port between I2C and SPI

  Returns: uint8_t from function call to writeRegisterSPI() or writeRegister I2C
*/
uint8_t selectMultiplexerPort(uint8_t port)
{
	uint8_t result = 0;

  if ((uint32_t) handPorts.imu[port] == SPI1_BASE)
  {
    printf("SELECTING IMU MULTIPLEXER PORT by SPI WRITE_REGISTER: IMU_NUM %d, PORT %d, ADDR 0x70, REG_ADDR %x",
      port, (int)handPorts.imu[port], 1 << port);
    result = writeRegisterSPI(handPorts.imu[port], I2C_MULTIPLEXER_ADDRESS, 1 << port);
	if (0 == result){
		converterInit();
	}
    return result;
  }
  else
  {
    printf("SELECTING IMU MULTIPLEXER PORT by I2C WRITE_REGISTER: IMU_NUM %d, PORT %d, ADDR 0x70, REG_ADDR %x",
      port, (int)handPorts.imu[port], 1 << port);
	  result = writeRegisterI2C(handPorts.imu[port], I2C_MULTIPLEXER_ADDRESS, 1 << port);
	  if (0 == result){
		  resetI2C(handPorts.imu[port]);
	  }
	  return result;
  }
}

uint8_t checkIMUStatus(uint8_t imuNumber){
  if (imuNumber >= NUM_IMUS)  // Error check
    return 0;

  return handStatus.imus[imuNumber];
}

/*
  Description: Abstracts writing the register of the IMU over I2C vs. SPI

  Returns: uint8_t result
*/
uint8_t writeRegisterIMU(uint32_t* port, uint8_t address, uint8_t registerAddress)
{
  uint8_t result = 0;

  if ((uint32_t) port == SPI1_BASE) {
    result = writeRegisterSPI(port, address, registerAddress);
	if (0 == result){
		converterInit();
	}
  }
  else {
    result = writeRegisterI2C(port, address, registerAddress);
	  if (0 == result){
		  resetI2C(port);
	  }
  }
  return result;
}

/*
  Description: Abstracts reading bytes from the IMU over I2C vs. SPI
  Takes in: port, address, number of bytes, values
  Returns: uint8_t result
*/
uint8_t readBytesIMU(uint32_t* port, uint8_t address, uint8_t numBytes, uint8_t* values)
{
  uint8_t result;

  if ((uint32_t) port == SPI1_BASE)
  {
    //printf("IMU_READING_BYTES by SPI READ: IMU Number %d, PORT %d, ADDR %x, length 8. ", 

    // printf("IMU_READING_BYTES by SPI READ: PORT %d, ADDR %x, length %d. \n", 
    //   (int)port, address, numBytes);
    result = readBytesSPI(port, address, numBytes, values);
	if (0 == result){
		converterInit();
	}
  }
  else
  {
    //printf("IMU_READING_BYTES by I2C READ: IMU Number %d, PORT %d, ADDR %x, length 8. ", 

    // printf("IMU_READING_BYTES by I2C READ: PORT %d, ADDR %x, length %d. \n", 
    //   (int)port, address, numBytes);
    result = readBytesI2C(port, address, numBytes, values);
	  if (0 == result){
		  resetI2C(port);
	  }
  }
  return result;
}

uint8_t writeMultiToRegisterIMU(uint32_t* port, uint8_t address, uint8_t registerAddress, uint8_t numBytes, uint8_t* data){

  uint8_t result;
  uint8_t msg[numBytes + 1];
  msg[0] = registerAddress;

  for (int i = 0; i < numBytes; i++){
    msg[i + 1] = data[i];
  }

  if ((uint32_t) port == SPI1_BASE)
  {
    //printf("IMU_READING_BYTES by SPI READ: IMU Number %d, PORT %d, ADDR %x, length 8. ", 

    // printf("IMU_READING_BYTES by SPI READ: PORT %d, ADDR %x, length %d. \n", 
    //   (int)port, address, numBytes);
    result = writeBytesSPI(port, address, msg, numBytes + 1, 0);
	if (0 == result){
		converterInit();
	}
  }
  else
  {
    //printf("IMU_READING_BYTES by I2C READ: IMU Number %d, PORT %d, ADDR %x, length 8. ", 

    // printf("IMU_READING_BYTES by I2C READ: PORT %d, ADDR %x, length %d. \n", 
    //   (int)port, address, numBytes);
    result = writeBytesI2C(port, address, msg, numBytes + 1, 0);
	if (0 == result){
	  resetI2C(port);
	}

  }
  return result;

}

// Used in loadCalData
void setCalibrationData(uint8_t* buffer){ 
  int i, j;
  printf("\nSetting Calibration Data...\n\n");
  // Iterate for each IMU

  // Debugging Buffer Data, For Loops Need Due to UART Buffer Capacity
  uint16_t *buffer_display = (uint16_t *)buffer;

  printf("Buffer_Display:");
  for (i = 0; i < (NUM_BNO055_OFFSET_REGISTERS / 2) * NUM_IMUS; i++){
    if (i % (NUM_BNO055_OFFSET_REGISTERS / 2) == 0){
      printf("\n");
    }
    printf("[%d],", buffer_display[i]);
  }
  printf("\n");

  for(i = 0; i < NUM_IMUS; i++){ 

      // Set operation mode to CONFIG_MODE
      setRegisterIMU(i, BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
      // Switching time from Other Modes to CONFIG Mode
      delay_ms(25);
      
      // Set each of the 22 calibration data registers
      for (j = 0; j < NUM_BNO055_OFFSET_REGISTERS; j++){     
        // ACCEL_OFFSET_X_LSB_ADDR is 0x55 and is the first calibration data register
        printf("IMU %d: 0x%x - [%d]\n", i, ACCEL_OFFSET_X_LSB_ADDR + j, buffer[NUM_BNO055_OFFSET_REGISTERS * i + j]);
        setRegisterIMU(i, ACCEL_OFFSET_X_LSB_ADDR + j, buffer[NUM_BNO055_OFFSET_REGISTERS * i + j]);
      }

      imu_cal_values_set[i] = 1;

      // After calibration data registers set, set operation mode to NDOF (type of fusion mode)
      setRegisterIMU(i, BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF);
      // Switching time from CONFIG Mode to Other Modes
      delay_ms(10);

  }

  memcpy((void *)handState.imus_calibration_data, (void *)buffer, 88);

  printf("\n\nLoaded Calibration Data\n");

}

void refreshCalibration(void){
  int i;
  for (i = 0; i < NUM_IMUS; i++){
    imu_cal_values_read[i] = 0;
  }
  for (i = 0; i < NUM_BNO055_OFFSET_REGISTERS / 2 * NUM_IMUS; i++)
  {
    handState.imus_calibration_data[i] = 0;
  }
  return;
}

/*
  Description: Updates IMU state machine
  Takes in: imuNumber 
  Returns: void
  
  IMU connections:
    0 -> Port 0 on the I2C Multiplexer
    1 -> Port 1 on the I2C Multiplexer
    2 -> Port 2 on the I2C Multiplexer
    3 -> Port 3 on the I2C Multiplexer
*/
void imu_poll_nonblocking_tick(const uint8_t imuNumber)
{

  imu_async_poll_state_t* state = (imu_async_poll_state_t*) & (imu_poll_state[imuNumber]);
  uint8_t values[32] = {0}; // 32 byte buffer for reads
  
  //uint8_t numBytesToRead;
  
  uint8_t result = 0;
  uint8_t registerAddress;
  // printf("IMU NUMBER: %d STATE: \n", imuNumber);

  if (checkIMUStatus(imuNumber) == 0)
    *state = IMU_STATE_WAIT;
  
  /*
  States:
    #1 IMU_STATE_SET_REGISTER -> 
    #2 IMU_STATE_READ_VALUES  -> 
        #2a IMU_DATA           ->
        #2b IMU_CAL_STATUS            ->
        #2c IMU_CAL_OFFSETS           ->
    #3 IMU_STATE_WAIT         ->
    DEFAULT                ->  
  */
  switch(*state)
  {
    // Selects register to read from
    case IMU_STATE_SET_REGISTER: 
      imu_state_count[imuNumber]++;
      // printf("IMU_STATE_SET_REGISTER\n");

      if (handPorts.multiplexer)
        selectMultiplexerPort(imuNumber);
      
      // Switch case to select register address to read data from
      switch(imu_poll_type[imuNumber]){
        case IMU_DATA:
          registerAddress = BNO055_QUATERNION_DATA_W_LSB_ADDR;
          break;
        case IMU_CAL_STATUS:
          registerAddress = BNO055_CALIB_STAT_ADDR;
          break;
        case IMU_CAL_OFFSETS:
          registerAddress = ACCEL_OFFSET_X_LSB_ADDR;
          break;
        default:
          registerAddress = BNO055_QUATERNION_DATA_W_LSB_ADDR;
          break;
      }

      // Writes register address
      result = writeRegisterIMU(handPorts.imu[imuNumber], handPorts.imuI2CAddress[imuNumber], registerAddress);

      // Check if register write worked. Redundant
      if (result) 
        *state = IMU_STATE_READ_VALUES;

      // TODO: Failed register write to result in fix attempt, possibly repeating set register state but may induce blocking

      *state = IMU_STATE_READ_VALUES;
      break;

    case IMU_STATE_READ_VALUES:
      // printf("IMU_STATE_READ_VALUES\n");
      
      if (handPorts.multiplexer)
        selectMultiplexerPort(imuNumber);

      switch(imu_poll_type[imuNumber]){
        case IMU_DATA:
          // printf("DATA\n");
          result = readBytesIMU(handPorts.imu[imuNumber], handPorts.imuI2CAddress[imuNumber], 8, values);

          // If register read succeeds, load quaternion data
          if (result){
            int imuDataBlock = imuNumber * NUM_QUATERNIONS;
            int i;
            for (i = 0; i < NUM_QUATERNIONS; i++)
              handState.imus[imuDataBlock + i] = (((uint16_t)values[(i * 2) + 1]) << 8) | ((uint16_t)values[i*2]);
          }

          // Check if calibration offsets have been read
          if (!imu_cal_values_read[imuNumber]){
            imu_poll_type[imuNumber] = IMU_CAL_STATUS;
            *state = IMU_STATE_SET_REGISTER;
          }
          else { // Exit state machine
            imu_poll_type[imuNumber] = IMU_DATA;
           *state = IMU_STATE_WAIT;
          }

          // writeRegisterIMU(handPorts.imu[imuNumber], handPorts.imuI2CAddress[imuNumber], ACCEL_OFFSET_X_LSB_ADDR);

          // result = readBytesIMU(handPorts.imu[imuNumber], handPorts.imuI2CAddress[imuNumber], 22, values);
          // // result = readCalibrationData(handPorts.imu[imuNumber], handPorts.imuI2CAddress[imuNumber], values);

          // printf("IMU %d: (%d),(%d)\n", imuNumber, values[0], values[1]);

          break;

        // Read single calibration status register
        case IMU_CAL_STATUS:
          
          result = readBytesIMU(handPorts.imu[imuNumber], handPorts.imuI2CAddress[imuNumber], 1, values);
          // printf("CAL_STAT for IMU %d: %d\n", imuNumber, values[0]);
          if (result)
            handState.imus_calibration_status[imuNumber] = values[0];
    
          if (handState.imus_calibration_status[imuNumber] == 0xFF && !imu_cal_values_set[imuNumber]){
            imu_cal_status_count[imuNumber]++;
            // printf("CAL_STATUS_COUNT IMU - %d: %d\n", imuNumber, imu_cal_status_count[imuNumber]);
          }
          else{
            imu_cal_status_count[imuNumber] = 0;
          }

          if (imu_cal_status_count[imuNumber] >= 100){
            imu_poll_type[imuNumber] = IMU_CAL_OFFSETS;
            *state = IMU_STATE_SET_REGISTER;
          }
          else { // Exit state machine
            imu_poll_type[imuNumber] = IMU_DATA;
           *state = IMU_STATE_WAIT;
          }
          break;

        case IMU_CAL_OFFSETS:  // There are 22 offset AND radius registers
          printf("-----------CAL_OFFSETS--------------------------\n");
          setRegisterIMU(imuNumber, BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
          // Switching time from Other Modes to CONFIG Mode
          delay_ms(25);

          writeRegisterIMU(handPorts.imu[imuNumber], handPorts.imuI2CAddress[imuNumber], ACCEL_OFFSET_X_LSB_ADDR);

          result = readBytesIMU(handPorts.imu[imuNumber], handPorts.imuI2CAddress[imuNumber], 22, values);
          // result = readCalibrationData(handPorts.imu[imuNumber], handPorts.imuI2CAddress[imuNumber], values);

          printf("\n(%x)\n", values[0]);

          setRegisterIMU(imuNumber, BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF);
          // Switching time from CONFIG Mode to Other Modes
          delay_ms(10);

          if (result){
            int i = 0;
            for (i = 0; i < 11; i++){
              handState.imus_calibration_data[imuNumber * 11 + i] = 
                  ((uint16_t)values[i * 2 + 1] << 8) | ((uint16_t)values[i * 2]);
            }

            imu_cal_values_read[imuNumber] = 1;

            printf("\nByte Data for IMU: %d\n", imuNumber);
            for (i = 0; i < NUM_BNO055_OFFSET_REGISTERS; i++){
              printf("[%d],", values[i]);
            }
            printf("\nCalibration Data for IMU: %d\n", imuNumber);
            for (i = 0; i < NUM_BNO055_OFFSET_REGISTERS / 2; i++){
              printf("[%d],", handState.imus_calibration_data[imuNumber * 11 + i]);
            }
            printf("\n");
          }
          
          // if (handState.imus_calibration_status[imuNumber] != 0xFF){
            imu_poll_type[imuNumber] = IMU_DATA;
            *state = IMU_STATE_SET_REGISTER;
            imu_cal_status_count[imuNumber] = 0;
          //}

          // else
          //   *state = IMU_STATE_WAIT;
          break;
          
        default:  // Do nothing as this should never occur
          *state = IMU_STATE_WAIT;
          break;
      }

      if (!result)  
        printf("ERROR. IMU %d\n", imuNumber);
        //handState.imus[imuNumber] = 0; // TODO: Change from handState.imus to handStatus.imus[]
      break;

    case IMU_STATE_WAIT:
      imu_state_count[imuNumber] = 0;
      imu_poll_type[imuNumber] = IMU_DATA;
      break;
    default:
      *state = IMU_STATE_WAIT;
      break;
  }
}
