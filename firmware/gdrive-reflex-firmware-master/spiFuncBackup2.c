#include "spiFunc.h"

void udelay(int utime)
{
  int mytime = SYSTIME;
  while(SYSTIME - mytime < utime);
}

int writeRegisterSPI(SPI_TypeDef* spiPort, uint8_t address, uint8_t registerAddress)
{
  uint8_t data[1] = {registerAddress};
  return writeBytesSPI(spiPort, address, data, 1, 0);
}

int writeBytesSPI(SPI_TypeDef* spiPort, uint8_t address, uint8_t* data, int len, int toggleAddress)
{
  GPIO_TypeDef *cs_gpio = GPIOA;
  uint32_t cs_pin_mask = 1 << PORTA_BRIDGE0_CS;
  
  cs_gpio->BSRRH = cs_pin_mask;           // assert CS
  udelay(4);                              // delay 4us

  spiPort->DR = 0x00;                     // send write command 
  udelay(15);                             // delay 15us

  spiPort->DR = (uint8_t) len;            // send data len
  udelay(15);                             // delay 15us
  
  uint8_t addr;
  if (toggleAddress == 0)
    addr = address << 1;
  else
    addr = address;                          
  spiPort->DR = ((uint8_t) addr);         // send addr
  udelay(15);                             // delay 15us

  for (int i = 0; i < len; i++)
  {
    spiPort->DR = data[i];                // send data[i]
    udelay(15);                           // delay 15us
  }

  cs_gpio->BSRRL = cs_pin_mask;           // de-assert CS

  if (len == 0 || data == NULL)
    return 0;
  const uint32_t wait = 180 + 110 * len;
  udelay(wait);

  return 0;
}

#define SLEEP 15

uint8_t readSPIStatus(SPI_TypeDef * spiPort)
{
  GPIO_TypeDef *cs_gpio = GPIOA;
  uint32_t cs_pin_mask = 1 << PORTA_BRIDGE0_CS;
  uint8_t status;

  cs_gpio->BSRRH = cs_pin_mask;               // assert CS
  udelay(4);

  spiPort->DR = 0x21;                         // send read command       
  udelay(15);

  spiPort->DR = 0x04;                         // send status register address
  udelay(15);

  spiPort->DR = 0xFF;                         // send dummy data
  udelay(30);

  status = spiPort->DR;
  udelay(15);

  cs_gpio->BSRRL = cs_pin_mask;               // de-assert CS
  udelay(4);

  return status;
}

int readBytesSPIAssert(SPI_TypeDef* spiPort, uint8_t address, uint8_t numBytes, uint8_t* values)
{
  GPIO_TypeDef *cs_gpio = GPIOA;
  uint32_t cs_pin_mask = 1 << PORTA_BRIDGE0_CS;

  cs_gpio->BSRRH = cs_pin_mask;               // assert CS
  // udelay(4);                                  // delay 4us
  
  spiPort->DR = 0x01;                         // send read command       
  udelay(SLEEP);                                 // delay 15us

  spiPort->DR = (uint8_t) numBytes;           // send data len
  udelay(SLEEP);                                 // delay 15us
                                              
  spiPort->DR = ((uint8_t) address << 1) + 1; // send addr
  udelay(SLEEP);                                 // delay 15us
  
  cs_gpio->BSRRL = cs_pin_mask;               // de-assert CS

  if (numBytes == 0 || values == NULL)
    return 0;

  const uint32_t wait = 180 + 110 * numBytes;
  udelay(wait);
  
  cs_gpio->BSRRH = cs_pin_mask;               // assert CS
  udelay(4);

  spiPort->DR = 0x06;                         // read buffer command
  // spiPort->DR; 
  udelay(SLEEP);                                 // delay 15us
  for (int i=0; i<numBytes;i++)
  {
    values[i] = (uint8_t) spiPort->DR;
    // if (i != numBytes-1)
    //   spiPort->DR = 0x0;                      
    udelay(SLEEP);                               // delay 15us
  }
  cs_gpio->BSRRL = cs_pin_mask;               // de-assert CS
  udelay(30);

  return 0;
}

int readCommmand(SPI_TypeDef* spiPort,uint8_t address, uint8_t numBytes)
{
  GPIO_TypeDef *cs_gpio = GPIOA;
  uint32_t cs_pin_mask = 1 << PORTA_BRIDGE0_CS;

  cs_gpio->BSRRH = cs_pin_mask;               // assert CS
  udelay(4);
  
  spiPort->DR = 0x01;                         // send read command
  // while((spiPort->SR & (SPI_SR_TXE)));        //while((spiPort->SR & (SPI_SR_BSY)));
  udelay(15);
  
  spiPort->DR = (uint8_t) numBytes;           // send data len                            
  // while((spiPort->SR & (SPI_SR_BSY)));
  udelay(15);
  
  spiPort->DR = ((uint8_t) address << 1) + 1; // send addr
  // while((spiPort->SR & (SPI_SR_BSY))); 
  udelay(15);
  
  cs_gpio->BSRRL = cs_pin_mask;               // de-assert CS

  return 0;
}


int readBytesSPI(SPI_TypeDef* spiPort, uint8_t address, uint8_t numBytes, uint8_t* values)
{
  GPIO_TypeDef *cs_gpio = GPIOA;
  uint32_t cs_pin_mask = 1 << PORTA_BRIDGE0_CS;
  // uint8_t status;

  readCommmand(spiPort, address, numBytes);

  uint32_t wait = 180 + 110 * numBytes;
  udelay(wait);

  if (numBytes == 0 || values == NULL)
    return 0;
  
  cs_gpio->BSRRH = cs_pin_mask;               // assert CS
  udelay(4);

  spiPort->DR = 0x06;                         // read buffer command
  // spiPort->DR; 
  udelay(SLEEP);                                 // delay 15us
  for (int i=0; i<numBytes;i++)
  {
    values[i] = (uint8_t) spiPort->DR;
    // if (i != numBytes-1)
    //   spiPort->DR = 0x0;                      
    udelay(SLEEP);                               // delay 15us
  }
  cs_gpio->BSRRL = cs_pin_mask;               // de-assert CS
  udelay(30);

  // status = (uint8_t) readSPIStatus(spiPort);
  // ledStatus(status);
  // while(1);

  return 0;
}

void ledStatus(uint8_t status)
{
  switch(status & 0xF)
  {
    case 0x0:
      ledsPattern(ON, ON, ON, ON);
      break;
    case 0x1:
      ledsPattern(ON, ON, ON, OFF);
      break;
    case 0x2:
      ledsPattern(ON, ON, OFF, ON);
      break;
    case 0x3:
      ledsPattern(ON, ON, OFF, OFF);
      break;
    case 0x4:
      ledsPattern(ON, OFF, ON, ON);
      break;
    case 0x5:
      ledsPattern(ON, OFF, ON, OFF);
      break;
    case 0x6:
      ledsPattern(ON, OFF, OFF, ON);
      break;
    case 0x7:
      ledsPattern(ON, OFF, OFF, OFF);
      break;
    case 0x8:
      ledsPattern(OFF, ON, ON, ON);
      break;
    case 0x9:
      ledsPattern(OFF, ON, ON, OFF);
      break;
    case 0xA:
      ledsPattern(OFF, ON, OFF, ON);
      break;
    case 0xB: // 1011 -> 011 3
      ledsPattern(OFF, ON, OFF, OFF);
      break;
    case 0xC:
      ledsPattern(OFF, OFF, ON, ON);
      break;
    case 0xD: // 1101 -> 101 -> 5
      ledsPattern(OFF, OFF, ON, OFF);
      break;
    case 0xE:
      ledsPattern(OFF, OFF, OFF, ON);
      break;
    case 0xF:
      ledsPattern(OFF, OFF, OFF, OFF);
      break;
    default:
      ledsPattern(OFF, OFF, OFF, OFF);
      break;
  }
}

void ledsPattern(int led1, int led2, int led3, int led4)
{
  int leds[4] = {led1, led2, led3, led4};

  for(int i=0; i<4;i++)
  {
    if (leds[i] == ON)
    {
      leds_on(i);
    }
    else
    {
      leds_off(i);
    }
  }
}