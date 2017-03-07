#include "i2cFunc.h"

#define MAX_TIME 10000 // CORRECT

int writeBytesI2C(uint32_t* port, uint8_t address, uint8_t* data, int len, int toggleAddress)
{
  // printf("initial time: %ld\n", SYSTIME);
  volatile int initialTime = SYSTIME;
  // printf("initial time: %d\n", initialTime);
  
  volatile int finalTime = SYSTIME;

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

  // printf("a\n");
  // while (!(i2cPort->SR1 & I2C_SR1_SB));
  while (!(i2cPort->SR1 & I2C_SR1_SB) && (finalTime - initialTime < MAX_TIME))
    finalTime = SYSTIME;
  // if (SYSTIME - initialTime > MAX_TIME)
  // {
  //   printf("start condition failed\n");
  //   i2cPort->CR1 |= I2C_CR1_STOP;
  //   udelay(10000);
  //   return 0;
  // }

  // Send address with write bit
  i2cPort->DR = ((uint8_t) addr); // puts the address to be sent on the buffer using global variable

  // printf("b\n");
  while(!(i2cPort->SR1 & (I2C_SR1_ADDR)) && (finalTime - initialTime < MAX_TIME))
    finalTime = SYSTIME;
  // if (SYSTIME - initialTime > MAX_TIME)
  // {
  //   printf("address not sent\n");
  //   return 0;
  // }
  i2cPort->SR2; // un-stretch clock by reading here (?)

  // printf("c\n");
  for (int i = 0; i < len; ++i)
  {
      i2cPort->DR = data[i]; // Send the address of the desired register
      while (!(i2cPort->SR1 & (I2C_SR1_BTF | I2C_SR1_AF)) && (finalTime - initialTime < MAX_TIME))
        finalTime = SYSTIME;
  }

  i2cPort->CR1 |= I2C_CR1_STOP;

  // printf("d\n");
  while (isBusyI2CPort(port) && (finalTime - initialTime < MAX_TIME))
    finalTime = SYSTIME;
  // if (SYSTIME - initialTime > MAX_TIME)
  // {
  //   printf("bus busy\n");
  //   return 0;
  // }

  // printf("final time: %ld\n", SYSTIME);
  // printf("final time: %d\n", finalTime);
  // printf("e\n");
  // printf("result e: %d \n", finalTime - initialTime >= MAX_TIME);
  if (finalTime - initialTime >= MAX_TIME)
  {
    // printf("TIMEOUT: initialTime: %d ", initialTime);
    // printf("finalTime: %d\n", finalTime);
    return 0;
  }
  return 1;
}

int writeRegisterI2C(uint32_t* port, uint8_t address, uint8_t registerAddress)
{
  int initialTime = SYSTIME;
  int finalTime = SYSTIME;
  I2C_TypeDef *i2cPort = (I2C_TypeDef *) port;
  // starting i2c communication on I2c bus 3 
  i2cPort->CR1 |=  I2C_CR1_START; // generating start condition
  i2cPort->SR1 &= ~I2C_SR1_AF;    // clearing acknowledge

  while (!(i2cPort->SR1 & I2C_SR1_SB) && (finalTime - initialTime < MAX_TIME))
    finalTime = SYSTIME;
  // Send address with write bit
  i2cPort->DR = ((uint8_t) address << 1); // puts the address to be sent on the buffer using global variable

  while(!(i2cPort->SR1 & (I2C_SR1_ADDR)) && (finalTime - initialTime < MAX_TIME))
    finalTime = SYSTIME;

  i2cPort->SR2; // un-stretch clock by reading here (?)

  i2cPort->DR = registerAddress; // Send the address of the desired register
  while (!(i2cPort->SR1 & (I2C_SR1_BTF | I2C_SR1_AF)) && (finalTime - initialTime < MAX_TIME))
    finalTime = SYSTIME;

  i2cPort->CR1 |= I2C_CR1_STOP;

  while (isBusyI2CPort(port) && (finalTime - initialTime < MAX_TIME)) // wait until bus is not busy anymore
    finalTime = SYSTIME;    

  if (finalTime - initialTime >= MAX_TIME)
  {
    printf("initialTime: %d", initialTime);
    printf("finalTime: %d", finalTime);
    printf(" TIMEOUT\n");
    return 0;
  }
  return 1;
}


int readBytesI2C(uint32_t* port, uint8_t address, int numBytes, uint8_t* values)
{
  int initialTime = SYSTIME;
  int finalTime = SYSTIME;
  I2C_TypeDef *i2cPort = (I2C_TypeDef *) port;
  // Send the address with the read bit
  // starting I2C communication
  i2cPort->CR1 |=  I2C_CR1_START;
  i2cPort->SR1 &= ~I2C_SR1_AF;  

  while (!(i2cPort->SR1 & I2C_SR1_SB) && (finalTime - initialTime < MAX_TIME))
    finalTime = SYSTIME;

  // Send address with read bit
  i2cPort->DR = ((uint8_t) address << 1) + 1; // puts the address to be sent on the buffer using global variable

  while (!(i2cPort->SR1 & (I2C_SR1_ADDR)) && (finalTime - initialTime < MAX_TIME))
    finalTime = SYSTIME;
  i2cPort->SR2; // un-stretch clock by reading here (?)

  i2cPort->CR1 |=  I2C_CR1_ACK; // multi-byte read. Acknowledge enable

  for (int i = 0; i < numBytes-1; i++)
  {
    while (!(i2cPort->SR1 & I2C_SR1_RXNE) && (finalTime - initialTime < MAX_TIME))
      finalTime = SYSTIME;
    values[i] = i2cPort->DR;
    i2cPort->CR1 |=  I2C_CR1_ACK; // multi-byte read. Acknowledge enable
  }
  while (!(i2cPort->SR1 & I2C_SR1_RXNE) && (finalTime - initialTime < MAX_TIME))
    finalTime = SYSTIME;

  values[numBytes-1] = i2cPort->DR;
  i2cPort->CR1 &= ~I2C_CR1_ACK; // last read

  i2cPort->CR1 |= I2C_CR1_STOP;

  while (isBusyI2CPort(port) && (finalTime - initialTime < MAX_TIME))
    finalTime = SYSTIME;

  if (finalTime - initialTime >= MAX_TIME)
  {
    printf("initialTime: %d", initialTime);
    printf("finalTime: %d", finalTime);
    printf(" TIMEOUT\n");
    return 0;
  }
  return 1;
}

int isBusyI2CPort(uint32_t* port)
{
  I2C_TypeDef *i2cPort = (I2C_TypeDef *) port;
  return i2cPort->SR2 & I2C_SR2_BUSY;
}