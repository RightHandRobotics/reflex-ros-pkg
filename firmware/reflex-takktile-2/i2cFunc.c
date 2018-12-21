#include "i2cFunc.h"

uint8_t writeBytesI2C(uint32_t* port, uint8_t address, uint8_t* data, int len, int toggleAddress)
{
  volatile int initialTime = SYSTIME;

  I2C_TypeDef *i2cPort = (I2C_TypeDef *) port;

  uint8_t addr;
  if (toggleAddress == 0)
    addr = address << 1;
  else
    addr = address;                          

  // Send the address with the read bit
  // starting I2C communication
  i2cPort->CR1 |=  I2C_CR1_START;
  i2cPort->SR1 &= ~I2C_SR1_AF;

  
  while (!(i2cPort->SR1 & I2C_SR1_SB) && (SYSTIME - initialTime < I2C_TIMEOUT));

  // Send address with write bit
  i2cPort->DR = ((uint8_t) addr); // puts the address to be sent on the buffer using global variable

  while(!(i2cPort->SR1 & (I2C_SR1_ADDR)) && (SYSTIME - initialTime < I2C_TIMEOUT));

  i2cPort->SR2; // un-stretch clock by reading here (?)

  for (int i = 0; i < len; ++i)
  {
      i2cPort->DR = data[i]; // Send the address of the desired register
      while (!(i2cPort->SR1 & (I2C_SR1_BTF | I2C_SR1_AF)) && (SYSTIME - initialTime < I2C_TIMEOUT));
  }
  i2cPort->CR1 |= I2C_CR1_STOP;

  while (isBusyI2CPort(port) && (SYSTIME - initialTime < I2C_TIMEOUT));

  if (SYSTIME - initialTime >= I2C_TIMEOUT)
  {
    return 0;
  }
  // static int maxTime = 0;
  // maxTime = (SYSTIME - initialTime) > maxTime ? (SYSTIME - initialTime) : maxTime;
  return 1;
}

uint8_t writeRegisterI2C(uint32_t* port, uint8_t address, uint8_t registerAddress)
{
  int initialTime = SYSTIME;

  I2C_TypeDef *i2cPort = (I2C_TypeDef *) port;
  // starting i2c communication on I2c bus 3 
  i2cPort->CR1 |=  I2C_CR1_START; // generating start condition
  i2cPort->SR1 &= ~I2C_SR1_AF;    // clearing acknowledge

  while (!(i2cPort->SR1 & I2C_SR1_SB) && (SYSTIME - initialTime < I2C_TIMEOUT)); 
  // Send address with write bit
  i2cPort->DR = ((uint8_t) address << 1); // puts the address to be sent on the buffer using global variable

  while(!(i2cPort->SR1 & (I2C_SR1_ADDR)) && (SYSTIME - initialTime < I2C_TIMEOUT));

  i2cPort->SR2; // un-stretch clock by reading here (?)

  i2cPort->DR = registerAddress; // Send the address of the desired register
  while (!(i2cPort->SR1 & (I2C_SR1_BTF | I2C_SR1_AF)) && (SYSTIME - initialTime < I2C_TIMEOUT));

  i2cPort->CR1 |= I2C_CR1_STOP;

  while (isBusyI2CPort(port) && (SYSTIME - initialTime < I2C_TIMEOUT)); // wait until bus is not busy anymore  

  if (SYSTIME - initialTime >= I2C_TIMEOUT)
  {
    return 0;
  }
  // static int maxTime = 0;
  // maxTime = (SYSTIME - initialTime) > maxTime ? (SYSTIME - initialTime) : maxTime;
  return 1;
}

uint8_t setRegisterI2C(uint32_t* port, uint8_t address, uint8_t registerAddress, uint8_t data)
{
  uint8_t msg[2] = {registerAddress, data};
  return writeBytesI2C(port, address, msg, 2, 0);
}

uint8_t readBytesI2C(uint32_t* port, uint8_t address, int numBytes, uint8_t* values)
{
  int initialTime = SYSTIME;

  I2C_TypeDef *i2cPort = (I2C_TypeDef *) port;
  // Send the address with the read bit
  // starting I2C communication
  i2cPort->CR1 |=  I2C_CR1_START;
  i2cPort->SR1 &= ~I2C_SR1_AF;  

  while (!(i2cPort->SR1 & I2C_SR1_SB) && (SYSTIME - initialTime < I2C_TIMEOUT));

  // Send address with read bit
  i2cPort->DR = ((uint8_t) address << 1) + 1; // puts the address to be sent on the buffer using global variable

  while (!(i2cPort->SR1 & (I2C_SR1_ADDR)) && (SYSTIME - initialTime < I2C_TIMEOUT));
  i2cPort->SR2; // un-stretch clock by reading here (?)

  i2cPort->CR1 |=  I2C_CR1_ACK; // multi-byte read. Acknowledge enable

  for (int i = 0; i < numBytes-1; i++)
  {
    while (!(i2cPort->SR1 & I2C_SR1_RXNE) && (SYSTIME - initialTime < I2C_TIMEOUT));
    values[i] = i2cPort->DR;
    i2cPort->CR1 |=  I2C_CR1_ACK; // multi-byte read. Acknowledge enable
  }
  while (!(i2cPort->SR1 & I2C_SR1_RXNE) && (SYSTIME - initialTime < I2C_TIMEOUT));

  values[numBytes-1] = i2cPort->DR;
  i2cPort->CR1 &= ~I2C_CR1_ACK; // last read

  i2cPort->CR1 |= I2C_CR1_STOP;

  while (isBusyI2CPort(port) && (SYSTIME - initialTime < I2C_TIMEOUT));

  if (SYSTIME - initialTime >= I2C_TIMEOUT)
  {
    return 0;
  }
  return 1;
}

uint8_t isBusyI2CPort(uint32_t* port)
{
  I2C_TypeDef *i2cPort = (I2C_TypeDef *) port;
  return i2cPort->SR2 & I2C_SR2_BUSY;
}

uint8_t resetI2C(uint32_t* port)
{
	switch ((uint32_t) port)
	{
	case I2C1_BASE:
		I2C1->CR1 |= (1<<15);
		delay_ms(1);
		I2C1->CR1 = 0;
		delay_ms(1);

		I2C1->CR2   |= APB_MHZ; // 42MHz -> 101010
		I2C1->CCR   |= I2C_CCR; // I2C_CCR = 210
		I2C1->TRISE &= ~0x3f; // 0x3f = 0b00111111 -> 0b11000000, clearing register
		I2C1->TRISE |= I2C_TRISE; // = I2C_TRISE = (APB_MHZ * 200 / 1000 + 1), maximum rise time
		I2C1->CR1   |= I2C_CR1_PE; // peripheral enable
		break;

	case I2C3_BASE:
		I2C3->CR1 |= (1<<15);
		delay_ms(1);
		I2C3->CR1 = 0;
		delay_ms(1);

		I2C3->CR2   |= APB_MHZ;
		I2C3->CCR   |= I2C_CCR;
		I2C3->TRISE &= ~0x3f;
		I2C3->TRISE |= I2C_TRISE;
		I2C3->CR1   |= I2C_CR1_PE;
		break;
	}
	return 1;
}
