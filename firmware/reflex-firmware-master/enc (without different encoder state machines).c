#include "enc.h"


// GLOBAL VARIABLES
enc_async_poll_state_t enc_poll_state = EPS_DONE;

void encInit()
{
  // change encoder I2C address
  // uint8_t data[2] = {0x15, 0x04}; // 0b0000100 = 0x04 0b0010000 = 0x10 
  // writeBytesSPI(SPI1, AS5048_7BIT_ADDRESS, data, 2, 0);

  // initializing encoders state
  for (int i = 0; i < NUM_ENC; i++)
  {
    handState.encoders[i] = 0;
  }
}

/*
void enc_poll_nonblocking_tick(const uint8_t bogus __attribute__((unused)))
  Description: Reads encoders values.

  Returns: void

  Encoders connections:
    0 -> Port I2C1
    1 -> Port SPI
    2 -> Port I2C3
*/
void enc_poll_nonblocking_tick(const uint8_t encoderNumber)
{
  static uint8_t i2cPort1Updated = 0;
  static uint8_t i2cPort3Updated = 0;
  int result;

  switch(enc_poll_state)
  {
    case EPS_DONE:
        // printf("encoder\n");
        result = setEncoderRegister(1, AS5048B_ANGLLSB_REG, SPI_TIMEOUT);
        result = readEncoder(1, SPI_TIMEOUT); // (spiPort, encoderNumber, timeout)
        
        if (isBusyI2CPort(handPorts.encoder[0]) && isBusyI2CPort(handPorts.encoder[2]))
        {
          // printf("neither 0 nor 2\n");
          enc_poll_state = EPS_I2C;
          i2cPort1Updated = 0;
          i2cPort3Updated = 0;
        }
        else if (!isBusyI2CPort(handPorts.encoder[0]) && !isBusyI2CPort(handPorts.encoder[2]))
        {
          // printf("0 and 2, EPS_DONE\n");
          result = setEncoderRegister(0, AS5048B_ANGLLSB_REG, I2C_TIMEOUT);
          result = readEncoder(0, I2C_TIMEOUT);  // (i2cPort, encoderNumber, timeout)
          result = setEncoderRegister(2, AS5048B_ANGLLSB_REG, I2C_TIMEOUT);
          result = readEncoder(2, I2C_TIMEOUT);  // (i2cPort, encoderNumber, timeout)
          enc_poll_state = EPS_DONE;
        } 
        else if (!isBusyI2CPort(handPorts.encoder[2]))
        {
          // printf("0\n");
          result = setEncoderRegister(0, AS5048B_ANGLLSB_REG, I2C_TIMEOUT);
          result = readEncoder(0, I2C_TIMEOUT);  // (i2cPort, encoderNumber, timeout)
          enc_poll_state = EPS_DONE;
        }
        else if (!isBusyI2CPort(handPorts.encoder[0]))
        {
          // printf("2\n");
          result = setEncoderRegister(2, AS5048B_ANGLLSB_REG, I2C_TIMEOUT);
          result = readEncoder(2, I2C_TIMEOUT);  // (i2cPort, encoderNumber, timeout)
          enc_poll_state = EPS_DONE;
        }      
      break;
    case EPS_I2C:
        if (!isBusyI2CPort(handPorts.encoder[0]) && !isBusyI2CPort(handPorts.encoder[2]))
        {
          // printf("0 and 2, EPS_I2C\n");
          result = setEncoderRegister(0, AS5048B_ANGLLSB_REG, I2C_TIMEOUT);
          result = readEncoder(0, I2C_TIMEOUT);  // (i2cPort, encoderNumber, timeout)
          result = setEncoderRegister(2, AS5048B_ANGLLSB_REG, I2C_TIMEOUT);
          result = readEncoder(2, I2C_TIMEOUT);  // (i2cPort, encoderNumber, timeout)
          enc_poll_state = EPS_DONE;
        } 
        else if (!isBusyI2CPort(handPorts.encoder[2]) && i2cPort1Updated == 0)
        {
          // printf("0\n");
          result = setEncoderRegister(0, AS5048B_ANGLLSB_REG, I2C_TIMEOUT);
          result = readEncoder(0, I2C_TIMEOUT);  // (i2cPort, encoderNumber, timeout)
          i2cPort1Updated = 1;
          if (i2cPort3Updated == 1)
            enc_poll_state = EPS_DONE;
        }
        else if (!isBusyI2CPort(handPorts.encoder[0]) && i2cPort3Updated == 0)
        {
          // printf("2\n");
          result = setEncoderRegister(2, AS5048B_ANGLLSB_REG, I2C_TIMEOUT);
          result = readEncoder(2, I2C_TIMEOUT);  // (i2cPort, encoderNumber, timeout)
          i2cPort3Updated = 1;
          if (i2cPort1Updated == 1)
            enc_poll_state = EPS_DONE;
        }
      break;
    default:
      enc_poll_state = EPS_DONE; // shouldn't get here
      break;

      result = result + 1 - 1;
      // printf("%d\n", result); // rmelo19, to correct, change to error checking.
  }
}

int checkTimeout(int utime, int initialTime)
{
 return (SYSTIME - initialTime > utime);
}

int setEncoderRegister(uint8_t encoderNumber, uint8_t encoderRegister,int timeout)
{
  switch ((uint32_t) handPorts.encoder[encoderNumber])
  {
    case I2C1_BASE:
      writeRegisterI2C(handPorts.encoder[encoderNumber], handPorts.encoderI2CAddress[encoderNumber], encoderRegister);
    break;
    case I2C3_BASE:
      writeRegisterI2C(handPorts.encoder[encoderNumber], handPorts.encoderI2CAddress[encoderNumber], encoderRegister);
    break;
    case SPI1_BASE:
      writeRegisterSPI(handPorts.encoder[encoderNumber], handPorts.encoderI2CAddress[encoderNumber], encoderRegister);
    break;
  }

  
  return 1;
}

int readEncoder(uint8_t encoderNumber, int timeout)
{
  // printf("Encoder %d: ", encoderNumber);
  uint8_t valueRead[2];
  switch ((uint32_t) handPorts.encoder[encoderNumber])
  {
    case I2C1_BASE:
      readBytesI2C(handPorts.encoder[encoderNumber], handPorts.encoderI2CAddress[encoderNumber], 2, valueRead);
      // printf(" I2C ");
    break;
    case I2C3_BASE:
      readBytesI2C(handPorts.encoder[encoderNumber], handPorts.encoderI2CAddress[encoderNumber], 2, valueRead);
      // printf(" I2C ");
    break;
    case SPI1_BASE:
      readBytesSPI(handPorts.encoder[encoderNumber], handPorts.encoderI2CAddress[encoderNumber], 2, valueRead);
      // printf(" SPI ");
    break;
  }
  handState.encoders[encoderNumber] = (((uint16_t) valueRead[0] << 6) + ((uint16_t) (valueRead[1] & 0x3F)));
  // printf("%d\n", handState.encoders[encoderNumber]);
  return 1;
}