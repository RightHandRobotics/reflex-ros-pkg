#include "enc.h"

// GLOBAL ALL FILES VARIABLE
enc_async_poll_state_t enc_poll_state[3] = {STATE_WAIT, STATE_WAIT, STATE_WAIT};

void encInit()
{
  // Initalize encoder state
  for (int i = 0; i < NUM_ENC; i++)
    handState.encoders[i] = 0;
}

/*
  Description: Updates state machine, which has 3 states:
    ENCODER_STATE_SET_REGISTER: set the right encoder register to read from
    ENCODER_STATE_READ_VALUES: read encoder values
    ENCODER_STATE_WAIT: skips the encoder while other state machines are not finished

  Returns: void
  
  Encoder connections:
    0 -> Port I2C1
    1 -> Port SPI
    2 -> Port I2C3
*/
void enc_poll_nonblocking_tick(const uint8_t encoderNumber)
{
  enc_async_poll_state_t* state = (enc_async_poll_state_t*)&(enc_poll_state[encoderNumber]);
  
  switch(*state)
  {
    case ENCODER_STATE_SET_REGISTER:
      if (setEncoderRegister(encoderNumber, AS5048B_ANGLLSB_REG, SPI_TIMEOUT));
        *state = ENCODER_STATE_READ_VALUES;
      break;
    case ENCODER_STATE_READ_VALUES:
      if(readEncoderValues(encoderNumber, SPI_TIMEOUT));
        *state = ENCODER_STATE_WAIT;
      break;
    case ENCODER_STATE_WAIT:
      break;
    default:
      *state = ENCODER_STATE_WAIT;
      break;
  }
}

uint8_t setEncoderRegister(uint8_t encoderNumber, uint8_t encoderRegister,int timeout)
{
  // Set encoder register for reading to the one specified
  uint8_t result = 0;

  switch ((uint32_t) handPorts.encoder[encoderNumber])
  {
    case I2C1_BASE: // Finger 1
      result = writeRegisterI2C(handPorts.encoder[encoderNumber], handPorts.encoderI2CAddress[encoderNumber], encoderRegister);
    break;
    case I2C3_BASE: // Finger 3
      result = writeRegisterI2C(handPorts.encoder[encoderNumber], handPorts.encoderI2CAddress[encoderNumber], encoderRegister);
    break;
    case SPI1_BASE: // Finger 2
      result = writeRegisterSPI(handPorts.encoder[encoderNumber], handPorts.encoderI2CAddress[encoderNumber], encoderRegister);
    break;
  }

  return result;
}

uint8_t readEncoderValues(uint8_t encoderNumber, int timeout)
{
  // Read values from set register
  uint8_t result = 0;
  uint8_t valueRead[2];

  switch ((uint32_t) handPorts.encoder[encoderNumber])
  {
    case I2C1_BASE: // Finger 1
      result = readBytesI2C(handPorts.encoder[encoderNumber], handPorts.encoderI2CAddress[encoderNumber], 2, valueRead);
    break;
    case I2C3_BASE: // Finger 3
      result = readBytesI2C(handPorts.encoder[encoderNumber], handPorts.encoderI2CAddress[encoderNumber], 2, valueRead);
    break;
    case SPI1_BASE: // Finger 2
      result = readBytesSPI(handPorts.encoder[encoderNumber], handPorts.encoderI2CAddress[encoderNumber], 2, valueRead);
    break;
  }

  handState.encoders[encoderNumber] = (((uint16_t) valueRead[0] << 6) + ((uint16_t) (valueRead[1] & 0x3F)));
  
  return result;
}