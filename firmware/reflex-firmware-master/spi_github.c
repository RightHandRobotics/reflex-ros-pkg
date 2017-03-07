#include "spiFunc.h"

void resetConverter(void)
{
  // the two SPI-I2C bridges have their RESETs tied to a common MCU pin,
  // so this function will reset both of them
  GPIOC->BSRRH = 1 << PORTC_I2C_BRIDGE_RESET;
  udelay(1000);
  GPIOC->BSRRL = 1 << PORTC_I2C_BRIDGE_RESET;
  udelay(1000);
}

uint8_t checkConverterIsBusy (uint8_t utime)
{
  uint8_t status[1] = {0};
  readConverterRegister(SC18IS601_REGISTER_I2C_STATUS, status);
  uint32_t startTime = SYSTIME;
  while (status[0] == 0xF3 && SYSTIME - startTime < utime)
  {
    readConverterRegister(SC18IS601_REGISTER_I2C_STATUS, status);
  }
  if (status[0] == 0xF3)
  {
    resetConverter();
    return 0;
  }
  else
    return 1;
}

uint8_t writeConverterRegister(uint8_t registerAddress, uint8_t data)
{
  uint32_t startTime = SYSTIME;
  SPI_TypeDef *spiPort = SPI1;
  GPIO_TypeDef *cs_gpio = GPIOA;
  const uint8_t msg[3] = {SC18IS601_WRITE_REGISTER_COMMAND, registerAddress, data};
  uint32_t cs_pin_mask = 1 << PORTA_BRIDGE0_CS;

  cs_gpio->BSRRH = cs_pin_mask;           // assert CS
  udelay(4);                              // delay 4us

  spiPort->DR;
  udelay(15); 

  for (int i = 0; i < 3; ++i)
  {
    spiPort->DR = msg[i];                     // send write register command 
    while (!(spiPort->SR & SPI_SR_TXE) && (SYSTIME - startTime < SPI_TIMEOUT));       // wait for buffer room
    while (!(spiPort->SR & SPI_SR_RXNE) && (SYSTIME - startTime < SPI_TIMEOUT));
    while ((spiPort->SR & SPI_SR_BSY)  && (SYSTIME - startTime < SPI_TIMEOUT));
    spiPort->DR;
    udelay(15);                                // delay 15us
  }
  cs_gpio->BSRRL = cs_pin_mask;           // de-assert CS
  udelay(5);

  if (SYSTIME - startTime > SPI_TIMEOUT)
    return 0;
  return 1;
}
uint8_t readConverterRegister(uint8_t registerAddress, uint8_t *data)
{
  uint32_t startTime = SYSTIME;
  SPI_TypeDef *spiPort = SPI1;
  GPIO_TypeDef *cs_gpio = GPIOA;
  const uint8_t msg[3] = {SC18IS601_READ_REGISTER_COMMAND, registerAddress, 0x00};
  uint32_t cs_pin_mask = 1 << PORTA_BRIDGE0_CS;
  
  cs_gpio->BSRRH = cs_pin_mask; // assert CS
  udelay(4);

  spiPort->DR;
  udelay(15); 

  for (int i = 0; i < 3; ++i)
  {
    spiPort->DR = msg[i];      // send write register command 
    while (!(spiPort->SR & SPI_SR_TXE) && (SYSTIME - startTime < SPI_TIMEOUT));       // wait for buffer room
    while (!(spiPort->SR & SPI_SR_RXNE) && (SYSTIME - startTime < SPI_TIMEOUT));
    while ((spiPort->SR & SPI_SR_BSY)  && (SYSTIME - startTime < SPI_TIMEOUT));
    if (i == 2)
      data[0] = spiPort->DR; 
    else
      spiPort->DR;
    udelay(15);

  }
  cs_gpio->BSRRL = cs_pin_mask; // de-assert CS
  udelay(5);

  if (SYSTIME - startTime > SPI_TIMEOUT)
    return 0;
  return 1;
}

uint8_t writeRegisterSPI(uint32_t* port, uint8_t address, uint8_t registerAddress)
{
  uint8_t data[1] = {registerAddress};
  return writeBytesSPI(port, address, data, 1, 0);
}

uint8_t writeBytesSPI(uint32_t* port, uint8_t address, uint8_t* data, int len, int toggleAddress)
{
  SPI_TypeDef *spiPort = (SPI_TypeDef*) port;
  GPIO_TypeDef *cs_gpio = GPIOA;
  uint32_t cs_pin_mask = 1 << PORTA_BRIDGE0_CS;
  
  cs_gpio->BSRRH = cs_pin_mask;           // assert CS
  udelay(4);                              // delay 4us

  spiPort->DR = SC18IS601_WRITE_N_BYTES_COMMAND;                     // send write command 
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

  // 0x0C 0b0000 1100 original
  // 0x0C 0b0000 1100 no toggle (0x0C << 1) >> 1 = 0x0C
  // (0x0C >> 1) 0x06

  // 0xC0 0b1100 0000 original
  // 0x40 0b0100 0000 no toggle 0xC0 << 1 = 0b1000 0000 = 0x80; 0x80 >> 1 = 0x40
  // 0x60 0b0110 0000 with toggle 0xC0 >> 1 = 0x60
  for (int i = 0; i < len; i++)
  {
    spiPort->DR = data[i];                // send data[i]
    udelay(15);                           // delay 15us
  }

  cs_gpio->BSRRL = cs_pin_mask;           // de-assert CS

  if (len == 0 || data == NULL)
    return 1;
  const uint32_t wait = 180 + 110 * len;
  udelay(wait);

  return 1;
}

uint8_t readCommmand(SPI_TypeDef* spiPort, uint8_t address, uint8_t numBytes)
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

  return 1;
}


uint8_t readBytesSPI(uint32_t* port, uint8_t address, uint8_t numBytes, uint8_t* values)
{
  SPI_TypeDef *spiPort = (SPI_TypeDef*) port;
  // GPIOC->BSRRH = 1 << PORTC_I2C_BRIDGE_RESET;
  // udelay(100);
  // GPIOC->BSRRL = 1 << PORTC_I2C_BRIDGE_RESET;
  // udelay(100);
  // tactile_init();

  GPIO_TypeDef *cs_gpio = GPIOA;
  uint32_t cs_pin_mask = 1 << PORTA_BRIDGE0_CS;
  // uint8_t status;

  readCommmand(spiPort, address, numBytes);

  uint32_t wait = 180 + 110 * numBytes;
  udelay(wait);

  if (numBytes == 0 || values == NULL)
    return 1;
  
  cs_gpio->BSRRH = cs_pin_mask;               // assert CS
  udelay(4);

  spiPort->DR = 0x06;                         // read buffer command
  udelay(15);
  values[0] =  spiPort->DR;
  udelay(15);                                 // delay 15us
  for (int i=0; i<numBytes;i++)
  {
    // if (i != numBytes-1)
    spiPort->DR = 0x0;
    udelay(15);
    values[i] = (uint8_t) spiPort->DR;                     
    udelay(15);                               // delay 15us
  }
  cs_gpio->BSRRL = cs_pin_mask;               // de-assert CS
  udelay(30);

  // udelay(70);
  // status = (uint8_t) readSPIStatus(spiPort);
  // ledStatus(status);
  // udelay(100);
  // while(1);

  return 1;
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