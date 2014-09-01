#include "reflex.h"
#include "tactile.h"
#include "stm32f4xx.h"
#include <stdio.h>
#include "pin.h"
#include "state.h"
#include "systime.h"

/////////////////////////////////////////////////////////////////////////
// I2C SETUP
//
// i2c1 scl = pb6, alternate function 4
// i2c1 sda = pb7, alternate function 4
// i2c3 scl = pa8, alternate function 4
// i2c3 sda = pc9, alternate function 4

#define PORTB_I2C1_SCL 6
#define PORTB_I2C1_SDA 7

#define PORTA_I2C3_SCL 8
#define PORTC_I2C3_SDA 9

/////////////////////////////////////////////////////////////////////////
// SPI-TO-I2C BRIDGES SETUP
//
// bridge shared reset = pc14
// bridge0 cs = pa4
// bridge0 miso = pa6 via SPI1 on AF 5
// bridge0 mosi = pb5 via SPI1 on AF 5
// bridge0 sclk = pa5 via SPI1 on AF 5
// bridge0 int = pc15 
// bridge1 cs = pb9
// bridge1 miso = pc2 via SPI2 on AF 5
// bridge1 mosi = pc3 via SPI2 on AF 5
// bridge1 sclk = pd3 via SPI2 on AF 5
// bridge1 int = pa0

#define PORTC_I2C_BRIDGE_RESET 14
#define PORTA_BRIDGE0_CS        4
#define PORTA_BRIDGE0_MISO      6
#define PORTB_BRIDGE0_MOSI      5
#define PORTA_BRIDGE0_SCLK      5
#define PORTC_BRIDGE0_INT      15
#define PORTB_BRIDGE1_CS        9
#define PORTC_BRIDGE1_MISO      2
#define PORTC_BRIDGE1_MOSI      3
#define PORTD_BRIDGE1_SCLK      3
#define PORTA_BRIDGE1_INT       0

#define BAROM_ADDR          0xC0
#define BCAST_ENABLE_ADDR   0x0C
#define BCAST_DISABLE_ADDR  0x0D

// our APB frequency is 42 mhz
#define APB_MHZ 42

// for 100 kHz i2c: 42 mhz / (2 * 100 khz) = 210
// for 400 kHz i2c: 42 mhz / (2 * 400 khz) =  53
#define I2C_CCR 210
#define I2C_TRISE (APB_MHZ * 200 / 1000 + 1)
#define I2C_READ 1

typedef enum { I2C_FAIL, I2C_SUCCESS } tactile_i2c_result_t;

static void tactile_bridge_spi_txrx(const uint8_t bridge_idx,
                                    const uint8_t txrx_len,
                                    const uint8_t *txd,
                                    uint8_t *rxd);
static uint8_t tactile_bridge_read_reg(const uint8_t bridge_idx,
                                       const uint8_t reg_idx);
static tactile_i2c_result_t tactile_bridge_i2c_write(const uint8_t bridge_idx,
                                     const uint8_t i2c_addr,
                                     const uint8_t tx_len,
                                     const uint8_t *txd);
static tactile_i2c_result_t 
tactile_bridge_i2c_read(const uint8_t bridge_idx,
                        const uint8_t i2c_addr,
                        const uint8_t rx_len,
                        uint8_t *rxd);
void tactile_bridge_reset();

static const uint8_t g_tactile_finger_addrs[SENSORS_PER_FINGER] =
{ 0x00, 0x02, 0x04, 0x06, 0x08, 0x10, 0x12, 0x14, 0x16 };

static const uint8_t g_tactile_palm_addrs[NUM_PALM_SENSORS] =
{ 0x60, 0x62, 0x64, 0x66, 0x68, 0x70, 0x72, 0x74, 0x76, 0x78, 0x7a };

static const uint8_t g_tactile_sensors_per_port[NUM_TACTILE_PORTS] =
{ SENSORS_PER_FINGER, SENSORS_PER_FINGER, SENSORS_PER_FINGER,
  NUM_PALM_SENSORS };

tactile_i2c_result_t tactile_i2c(const uint8_t tactile_port, 
                                 const uint8_t address,
                                 uint8_t *data,
                                 const uint8_t data_len);

//////////////////////////////////////////////////////////////////////////////

void tactile_init()
{
  printf("tactile_init()\r\n");
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                  RCC_AHB1ENR_GPIOBEN |
                  RCC_AHB1ENR_GPIOCEN |
                  RCC_AHB1ENR_GPIODEN;
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN |
                  RCC_APB1ENR_I2C3EN |
                  RCC_APB1ENR_SPI2EN;
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

  pin_set_alternate_function(GPIOB, PORTB_I2C1_SCL, 4);
  pin_set_alternate_function(GPIOB, PORTB_I2C1_SDA, 4);
  pin_set_output_type(GPIOB, PORTB_I2C1_SCL, PIN_OUTPUT_TYPE_OPEN_DRAIN);
  pin_set_output_type(GPIOB, PORTB_I2C1_SDA, PIN_OUTPUT_TYPE_OPEN_DRAIN);

  pin_set_alternate_function(GPIOA, PORTA_I2C3_SCL, 4);
  pin_set_alternate_function(GPIOC, PORTC_I2C3_SDA, 4);
  pin_set_output_type(GPIOA, PORTA_I2C3_SCL, PIN_OUTPUT_TYPE_OPEN_DRAIN);
  pin_set_output_type(GPIOC, PORTC_I2C3_SDA, PIN_OUTPUT_TYPE_OPEN_DRAIN);

  //I2C1->CCR |= I2C_CCR_FS | // set fast mode
  //             35; // 42 MHz / (3 * 400 kHz) == 35
  //I2C1->TRISE = 42 * 300 / 1000 + 1; // not sure here.
  I2C1->CR2   |= APB_MHZ; 
  I2C1->CCR   |= I2C_CCR; 
  I2C1->TRISE &= ~0x3f;
  I2C1->TRISE |= I2C_TRISE;
  I2C1->CR1   |= I2C_CR1_PE;

  I2C3->CR2   |= APB_MHZ; 
  I2C3->CCR   |= I2C_CCR;
  I2C3->TRISE &= ~0x3f;
  I2C3->TRISE |= I2C_TRISE;
  I2C3->CR1   |= I2C_CR1_PE;

  // now, set up the spi-to-i2c bridges
  pin_set_output(GPIOC, PORTC_I2C_BRIDGE_RESET);
  pin_set_output(GPIOA, PORTA_BRIDGE0_CS);
  pin_set_output(GPIOB, PORTB_BRIDGE1_CS);
  pin_set_alternate_function(GPIOA, PORTA_BRIDGE0_MISO, 5);
  pin_set_alternate_function(GPIOB, PORTB_BRIDGE0_MOSI, 5);
  pin_set_alternate_function(GPIOA, PORTA_BRIDGE0_SCLK, 5);
  pin_set_alternate_function(GPIOC, PORTC_BRIDGE1_MISO, 5);
  pin_set_alternate_function(GPIOC, PORTC_BRIDGE1_MOSI, 5);
  pin_set_alternate_function(GPIOD, PORTD_BRIDGE1_SCLK, 5);

  // spi1 is running from a 84 MHz pclk. set it up with 
  // sclk = pclk/128 to stay within datasheet limits.
  SPI1->CR1 = SPI_CR1_BR_2 |
              SPI_CR1_BR_1 |
              SPI_CR1_MSTR |
              SPI_CR1_CPOL |
              SPI_CR1_CPHA |
              SPI_CR1_SSM  |
              SPI_CR1_SSI  |
              SPI_CR1_SPE;

  // bit rate = 42 mhz / 64 = 750 kilobit
  SPI2->CR1 = SPI_CR1_BR_2 |
              SPI_CR1_BR_0 |
              SPI_CR1_MSTR |
              SPI_CR1_CPOL |
              SPI_CR1_CPHA |
              SPI_CR1_SSM  |
              SPI_CR1_SSI  |
              SPI_CR1_SPE;

  pin_set_output_level(GPIOA, PORTA_BRIDGE0_CS, 1);
  pin_set_output_level(GPIOA, PORTB_BRIDGE1_CS, 1);

  tactile_bridge_reset();
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 6; j++)
      tactile_bridge_read_reg(i, j);

  /*
  for (int port = 0; port < NUM_TACTILE_PORTS; port++)
  {
    for (int sensor = 0; sensor < g_tactile_sensors_per_port[port]; sensor++)
    {
      const uint8_t mcu_addr = port < NUM_FINGERS ? 
                               g_tactile_finger_addrs[sensor] :
                               g_tactile_palm_addrs[sensor];
      tactile_i2c_result_t result = 
        tactile_i2c(port, mcu_addr, NULL, 0) == I2C_SUCCESS &&
        tactile_i2c(port, BAROM_ADDR, NULL, 0) == I2C_SUCCESS;
      printf("%s: port %d sensor %d at addr 0x%02x\r\n",
             result == I2C_SUCCESS ? "SUCCESS" : "FAIL",
             port, sensor, mcu_addr);
    }
  }
  */
  printf("done with tactile_init()\r\n");
}

void tactile_bridge_reset()
{
  // the two SPI-I2C bridges have their RESETs tied to a common MCU pin,
  // so this function will reset both of them
  printf("tactile_bridge_reset()\r\n");
  GPIOC->BSRRH = 1 << PORTC_I2C_BRIDGE_RESET;
  for (volatile int i = 0; i < 10000; i++) { } // assert RESET
  GPIOC->BSRRL = 1 << PORTC_I2C_BRIDGE_RESET;
  for (volatile int i = 0; i < 10000; i++) { } // then let them boot up
}

tactile_i2c_result_t tactile_i2c(uint8_t port,
                                 uint8_t address,
                                 uint8_t *data,
                                 uint8_t data_len)
{
  if (port == 0 || port == 1) // these ports are MCU on-chip I2C interfaces
  {
    /*
    printf("i2c xfer to port %d addr 0x%02x data len %d\r\n", 
           port, address, data_len);
    */
    I2C_TypeDef *i2c;
    if (port == 0)
      i2c = I2C1;
    else
      i2c = I2C3;
    i2c->CR1 |=  I2C_CR1_START;
    i2c->SR1 &= ~I2C_SR1_AF;
    while (!(i2c->SR1 & I2C_SR1_SB)) { }
    i2c->DR = address; 
    while (!(i2c->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF))) { }
    int address_fail = (i2c->SR1 & I2C_SR1_AF) ? 1 : 0;
    if (!data_len)
      i2c->CR1 |= I2C_CR1_STOP;
    i2c->SR2; // un-stretch clock by reading here (?)
    if (!address_fail && data_len)
    {
      if (!(address & 0x1))
      {
        // it's a write transaction
        for (int i = 0; i < data_len; i++)
        {
          i2c->DR = data[i];
          while (!(i2c->SR1 & (I2C_SR1_BTF | I2C_SR1_AF))) { }
          if (i2c->SR1 & I2C_SR1_AF)
            break; // didn't get an ACK
        }
      }
      else
      {
        // it's a read transaction
        //if (!data_len)
        //  i2c->CR1 &= ~I2C_CR1_ACK;
        for (int i = 0; i < data_len; i++)
        {
          if (i != data_len - 1)
            i2c->CR1 |= I2C_CR1_ACK;
          else
            i2c->CR1 &= ~I2C_CR1_ACK;
          while (!(i2c->SR1 & I2C_SR1_RXNE)) { } // wait for it...
          data[i] = i2c->DR;
        }
      }
    }
    i2c->CR1 |= I2C_CR1_STOP;
    while (i2c->SR2 & I2C_SR2_BUSY) { }
    for (volatile int i = 0; i < 3000; i++) { } // wait a bit
    return address_fail ? I2C_FAIL : I2C_SUCCESS;
  }
  else if (port == 2 || port == 3) // these ports are via SPI-I2C bridge chips
  {
    const uint8_t bridge_idx = port - 2;
    if (address & 0x1) // is it a read transaction?
      return tactile_bridge_i2c_read(bridge_idx, address, data_len, data);
    else
      return tactile_bridge_i2c_write(bridge_idx, address, data_len, data);
  }
  else
    return I2C_FAIL;
}

void tactile_poll()
{
  for (uint_fast8_t port = 0; port < NUM_TACTILE_PORTS; port++)
  {
    //printf("port %d time %12u\r\n", port, (unsigned)SYSTIME);
    for (uint_fast8_t sensor_idx = 0; 
         sensor_idx < g_tactile_sensors_per_port[port]; 
         sensor_idx++)
    {
      // tell the MCU we want to broadcast to everybody
      if (tactile_i2c(port, BCAST_ENABLE_ADDR, NULL, 0) != I2C_SUCCESS)
        continue;
      // tell everybody we want them to start their sampling process
      uint8_t msg[4] = { 0x12, 0x01, 0x00, 0x00};
      if (tactile_i2c(port, BAROM_ADDR, msg, 2) != I2C_SUCCESS)
        continue;
      // disable everybody by reading one byte...
      if (tactile_i2c(port, BCAST_DISABLE_ADDR, msg, 1) != I2C_SUCCESS)
        continue;
      uint8_t sensor_addr; // look up the sensor address
      if (port < NUM_FINGERS)
        sensor_addr = g_tactile_finger_addrs[sensor_idx];
      else
        sensor_addr = g_tactile_palm_addrs[sensor_idx];
      // activate this sensor
      if (tactile_i2c(port, sensor_addr, NULL, 0) != I2C_SUCCESS)
        continue;
      msg[0] = 0;
      // tell it we want to read the data
      if (tactile_i2c(port, BAROM_ADDR, msg, 1) != I2C_SUCCESS)
        continue;
      // now, actually read the data
      if (tactile_i2c(port, BAROM_ADDR | I2C_READ, msg, 4) != I2C_SUCCESS)
        continue;
      //printf("port %d mcu %d sensor %d rx 4 bytes: 0x%02x  0x%02x  0x%02x  0x%02x\r\n",
      //       port, mcu, sensor, msg[0], msg[1], msg[2], msg[3]);
      const uint16_t pressure = ((uint16_t)msg[0] << 2) | (msg[1] >> 6);
      const uint16_t temperature = ((uint16_t)msg[2] << 2) | (msg[3] >> 6);
      //printf("port %d sensor 0x%02x pressure %06d  temperature %06d\r\n",
      //       port, sensor_addr, pressure, temperature);

      const uint_fast8_t state_sensor_idx = port * SENSORS_PER_FINGER + 
                                            sensor_idx;
      g_state.tactile_pressures   [state_sensor_idx] = pressure;
      g_state.tactile_temperatures[state_sensor_idx] = temperature;

      // de-activate this sensor
      if (tactile_i2c(port, sensor_addr | I2C_READ, msg, 1) != I2C_SUCCESS)
          continue;
    }
  }
  //printf("\r\n");
}

static uint8_t tactile_bridge_read_reg(const uint8_t bridge_idx,
                                       const uint8_t reg_idx)
{
  uint8_t rxd[3] = {0};
  uint8_t txd[3] = {0x21, reg_idx, 0};
  tactile_bridge_spi_txrx(bridge_idx, 3, txd, rxd);
  const uint8_t reg_val = rxd[2];
  //printf("spi reg 0x%02x = 0x%02x\r\n", reg_idx, reg_val);
  return reg_val;
}

static void tactile_bridge_spi_txrx(const uint8_t bridge_idx,
                                    const uint8_t txrx_len,
                                    const uint8_t *txd,
                                    uint8_t *rxd)
{
  if (bridge_idx > 1)
    return;
  SPI_TypeDef *spi;
  GPIO_TypeDef *cs_gpio;
  uint32_t cs_pin_mask;
  if (bridge_idx == 0)
  {
    spi = SPI1;
    cs_gpio = GPIOA;
    cs_pin_mask = 1 << PORTA_BRIDGE0_CS;
  }
  else
  {
    spi = SPI2;
    cs_gpio = GPIOB;
    cs_pin_mask = 1 << PORTB_BRIDGE1_CS;
  }
  // assert CS 
  cs_gpio->BSRRH = cs_pin_mask;
  for (volatile int i = 0; i < 100; i++) { } // la di dah...
  spi->DR; // clear rx buffer
  for (uint8_t i = 0; i < txrx_len; i++)
  {
    spi->DR = txd[i]; // read internal register command
    while (!(spi->SR & SPI_SR_TXE)) { } // wait for buffer room
    while (!(spi->SR & SPI_SR_RXNE)) { }
    while (spi->SR & SPI_SR_BSY) { }
    if (rxd)
      rxd[i] = spi->DR; 
    else
      spi->DR;
    for (volatile int i = 0; i < 150; i++) { } // wait between SPI bytes
  }
  for (volatile int i = 0; i < 100; i++) { } // la di dah...
  cs_gpio->BSRRL = cs_pin_mask;
  for (volatile int i = 0; i < 100; i++) { } // la di dah...
}

tactile_i2c_result_t 
tactile_bridge_wait_for_completion(const uint8_t bridge_idx)
{
  for (volatile int i = 0; i < 1000; i++) { } // la di dah...
  uint8_t bridge_state = 0xf3;
  int wait_count = 0;
  while (bridge_state == 0xf3)
  {
    for (volatile int i = 0; i < 1000; i++) { } // la di dah...
    bridge_state = tactile_bridge_read_reg(bridge_idx, 0x4);
    if (++wait_count > 1000) 
    {
      // a SPI-I2C bridge locked up. try to reset them.
      tactile_bridge_reset();
      return I2C_FAIL;
    }
    //printf("bridge_state = 0x%02x\r\n", bridge_state);
  }
  if (bridge_state == 0xf0)
    return I2C_SUCCESS;
  else
    return I2C_FAIL;
}

static tactile_i2c_result_t tactile_bridge_i2c_write(const uint8_t bridge_idx,
                                     const uint8_t i2c_addr,
                                     const uint8_t tx_len,
                                     const uint8_t *txd)
{
  uint8_t trimmed_tx_len = tx_len;
  if (trimmed_tx_len > 250)
    trimmed_tx_len = 250;
  uint8_t msg[256];
  msg[0] = 0x00; // write i2c command
  msg[1] = trimmed_tx_len;
  msg[2] = i2c_addr; // will always perform a write... ignores the LSB

  for (int i = 0; i < trimmed_tx_len; i++)
    msg[i+3] = txd[i];
  tactile_bridge_spi_txrx(bridge_idx, trimmed_tx_len + 3, msg, NULL);
  return tactile_bridge_wait_for_completion(bridge_idx);
}

static tactile_i2c_result_t 
tactile_bridge_i2c_read(const uint8_t bridge_idx,
                        const uint8_t i2c_addr,
                        const uint8_t rx_len,
                        uint8_t *rxd)
{
  uint8_t msg[3];
  msg[0] = 0x01; // read i2c command
  msg[1] = rx_len;
  msg[2] = i2c_addr; // will always performa a read... ignores the LSB
  tactile_bridge_spi_txrx(bridge_idx, 3, msg, NULL);
  if (tactile_bridge_wait_for_completion(bridge_idx) == I2C_FAIL)
    return I2C_FAIL;
  uint8_t read_msg[rx_len+1], rx_msg[rx_len+1];
  read_msg[0] = 0x06; // read buffer command
  for (int i = 1; i < rx_len+1; i++)
    read_msg[i] = 0;
  tactile_bridge_spi_txrx(bridge_idx, rx_len+1, read_msg, rx_msg);
  for (int i = 1; i < rx_len+1; i++)
    rxd[i-1] = rx_msg[i];
  for (volatile int i = 0; i < 1000; i++) { } // la di dah...
  uint8_t bridge_state = tactile_bridge_read_reg(bridge_idx, 0x4);
  return (bridge_state == 0xf0 ? I2C_SUCCESS : I2C_FAIL);
}

