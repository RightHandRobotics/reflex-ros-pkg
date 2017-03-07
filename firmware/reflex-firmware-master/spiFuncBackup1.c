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

int readBytesSPI(SPI_TypeDef* spiPort, uint8_t address, uint8_t numBytes, uint8_t* values)
{
  GPIO_TypeDef *cs_gpio = GPIOA;
  uint32_t cs_pin_mask = 1 << PORTA_BRIDGE0_CS;

  cs_gpio->BSRRH = cs_pin_mask;               // assert CS
  udelay(4);                                  // delay 4us
  
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