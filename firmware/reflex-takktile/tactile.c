#include "reflex.h"
#include "tactile.h"
#include "stm32f4xx.h"
#include <stdio.h>
#include "pin.h"
#include "state.h"
#include "error.h"
#include "systime.h"
#include <string.h>
#include <stdbool.h>

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

tactile_async_poll_state_t tactile_poll_states[NUM_TACTILE_PORTS] =
  { TPS_IDLE, TPS_IDLE, TPS_IDLE, TPS_IDLE };

typedef enum { I2C_FAIL, I2C_SUCCESS } tactile_i2c_result_t;

static void tactile_bridge_spi_txrx(const uint8_t bridge_idx,
                                    const uint8_t txrx_len,
                                    const uint8_t *txd,
                                    uint8_t *rxd);
static uint8_t tactile_bridge_read_reg(const uint8_t bridge_idx,
                                       const uint8_t reg_idx);
static void tactile_bridge_write_reg(const uint8_t bridge_idx,
                                     const uint8_t reg_idx,
                                     const uint8_t reg_val);
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

#define TACTILE_I2C_SUCCESS 0xffffffff
#define TACTILE_I2C_FAIL    0xfffffffe

typedef enum
{
  TATS_START = 0,
  TATS_ADDR,
  TATS_WRITE,
  TATS_READ,
  TATS_STOP,
  TATS_STOP_WAIT,
  TATS_DONE_FAIL = TACTILE_I2C_FAIL,
  TATS_DONE_SUCCESS = TACTILE_I2C_SUCCESS
} tactile_internal_i2c_status_t;

typedef enum
{
  TBPS_REQUEST_CS_LOW = 0,
  TBPS_REQUEST_TX_CMD,
  TBPS_REQUEST_TX_LEN,
  TBPS_REQUEST_TX_ADDR,
  TBPS_TX_DATA,
  TBPS_WAIT_FOR_COMPLETION,
  TBPS_READ_CS_LOW,
  TBPS_READ_TXRX,
  TBPS_READ_TXRX_WAIT,
  TBPS_DONE_FAIL = TACTILE_I2C_FAIL,
  TBPS_DONE_SUCCESS = TACTILE_I2C_SUCCESS
} tactile_bridged_i2c_status_t;

static uint8_t g_tactile_i2c_async_address      [NUM_TACTILE_PORTS] = {0};
static uint8_t g_tactile_i2c_async_data         [NUM_TACTILE_PORTS][256];
static uint8_t g_tactile_i2c_async_data_txrx_idx[NUM_TACTILE_PORTS] = {0};
static uint8_t g_tactile_i2c_async_data_len     [NUM_TACTILE_PORTS] = {0};
static volatile uint32_t g_tactile_i2c_async_start_us[NUM_TACTILE_PORTS] = {0};

static tactile_internal_i2c_status_t
g_tactile_internal_i2c_status[NUM_INTERNAL_I2C] = { TATS_START, TATS_START };

static bool g_tactile_i2c_async_address_fail[NUM_INTERNAL_I2C] = {false};

tactile_bridged_i2c_status_t
g_tactile_bridged_i2c_status[NUM_BRIDGED_I2C] =
{ TBPS_REQUEST_CS_LOW, TBPS_REQUEST_CS_LOW };

void tactile_i2c_async_start(const uint8_t port, const uint8_t address,
                             uint8_t *data, const uint8_t data_len)
{
  if (port >= NUM_TACTILE_PORTS)
    return; // bogus

  g_tactile_i2c_async_address[port]  = address;
  g_tactile_i2c_async_data_len[port] = data_len;
  if (data)
    memcpy(g_tactile_i2c_async_data[port], data, data_len);
  else
    memset(g_tactile_i2c_async_data[port], 0, data_len);
  g_tactile_i2c_async_data_txrx_idx[port] = 0;

/*
  if (port == 2)
    printf("i2c port %d addr 0x%02x len %d\r\n", port, address, data_len);
*/

  if (port == 0 || port == 1)
  {
    I2C_TypeDef *i2c;
    if (port == 0)
      i2c = I2C1;
    else
      i2c = I2C3;
    i2c->CR1 |=  I2C_CR1_START;
    i2c->SR1 &= ~I2C_SR1_AF;
    g_tactile_internal_i2c_status[port] = TATS_START;
  }
  else if (port == 2 || port == 3) // use a bridge
  {
    const uint_fast8_t bridge = port - 2;
    GPIO_TypeDef *cs_gpio = NULL;
    uint32_t cs_pin_mask = 0;
    if (bridge == 0)
    {
      cs_gpio = GPIOA;
      cs_pin_mask = 1 << PORTA_BRIDGE0_CS;
    }
    else // bridge == 1
    {
      cs_gpio = GPIOB;
      cs_pin_mask = 1 << PORTB_BRIDGE1_CS;
    }
    // assert CS
    cs_gpio->BSRRH = cs_pin_mask;
    g_tactile_bridged_i2c_status[bridge] = TBPS_REQUEST_CS_LOW;
  }
  g_tactile_i2c_async_start_us[port] = SYSTIME;
}

void tactile_internal_i2c_async_tick(const uint_fast8_t port)
{
  if (port >= NUM_INTERNAL_I2C)
    return; // bogus
  tactile_internal_i2c_status_t *status = &g_tactile_internal_i2c_status[port];
  //static int tat_cnt = 0;
  I2C_TypeDef *i2c;
  if (port == 0)
    i2c = I2C1;
  else
    i2c = I2C3;
  /*
     if (port == 0 && ++tat_cnt % 50000 == 0)
     printf("i2c async tick port %d tats %d sr = 0x%08x\r\n",
     port, (int)*tats, (unsigned)i2c->SR1);
   */

  switch (*status)
  {
    case TATS_START:
      if (i2c->SR1 & I2C_SR1_SB)
      {
        i2c->DR = g_tactile_i2c_async_address[port];
        *status = TATS_ADDR;
      }
      break;
    case TATS_ADDR:
      if (i2c->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF))
      {
        bool address_fail = (i2c->SR1 & I2C_SR1_AF) ? true : false;
        g_tactile_i2c_async_address_fail[port] = address_fail;
        int no_payload = (0 == g_tactile_i2c_async_data_len[port]);
        if (no_payload)
          i2c->CR1 |= I2C_CR1_STOP; // this seemed needed... not sure why now.
        i2c->SR2; // un-stretch clock by reading here (?)
        if (!address_fail && !no_payload)
        {
          if (!(g_tactile_i2c_async_address[port] & 0x1))
          {
            // it's a write transaction
            g_tactile_i2c_async_data_txrx_idx[port] = 0;
            i2c->DR = g_tactile_i2c_async_data[port][0];
            *status = TATS_WRITE;
          }
          else
          {
            // it's a read transaction
            g_tactile_i2c_async_data_txrx_idx[port] = 0;
            if (g_tactile_i2c_async_data_len[port] == 1)
              i2c->CR1 &= ~I2C_CR1_ACK; // single-byte read
            else
              i2c->CR1 |=  I2C_CR1_ACK; // multi-byte read. ack this one.
            *status = TATS_READ;
          }
        }
        else
        {
          i2c->CR1 |= I2C_CR1_STOP;
          *status = TATS_STOP;
        }
      }
      break;
    case TATS_WRITE:
      if (i2c->SR1 & (I2C_SR1_BTF | I2C_SR1_AF))
      {
        if (i2c->SR1 & I2C_SR1_AF)
        {
          i2c->CR1 |= I2C_CR1_STOP;
          *status = TATS_STOP;
        }
        else
        {
          g_tactile_i2c_async_data_txrx_idx[port]++;
          if (g_tactile_i2c_async_data_txrx_idx[port] >=
              g_tactile_i2c_async_data_len[port])
          {
            i2c->CR1 |= I2C_CR1_STOP;
            *status = TATS_STOP;
          }
          else
          {
            const uint8_t txrx_idx = g_tactile_i2c_async_data_txrx_idx[port];
            i2c->DR = g_tactile_i2c_async_data[port][txrx_idx];
          }
        }
      }
      break;
    case TATS_READ:
      if (i2c->SR1 & I2C_SR1_RXNE)
      {
        const uint8_t txrx_idx = g_tactile_i2c_async_data_txrx_idx[port];
        g_tactile_i2c_async_data[port][txrx_idx] = i2c->DR;
        g_tactile_i2c_async_data_txrx_idx[port]++;
        if (g_tactile_i2c_async_data_txrx_idx[port] >=
            g_tactile_i2c_async_data_len[port])
        {
          i2c->CR1 |= I2C_CR1_STOP;
          *status = TATS_STOP;
        }
        else
        {
          if (g_tactile_i2c_async_data_len[port] - 1 ==
              g_tactile_i2c_async_data_txrx_idx[port])
            i2c->CR1 &= ~I2C_CR1_ACK; // last read
          else
            i2c->CR1 |=  I2C_CR1_ACK; // more reads to come. ack it.
        }
      }
      break;
    case TATS_STOP:
      if (!(i2c->SR2 & I2C_SR2_BUSY))
      {
        *status = TATS_STOP_WAIT; // wait a bit for the line to clear
        g_tactile_i2c_async_start_us[port] = SYSTIME;
      }
      break;
    case TATS_STOP_WAIT:
      if (SYSTIME - g_tactile_i2c_async_start_us[port] > 10)
      {
        if (g_tactile_i2c_async_address_fail[port])
          *status = TATS_DONE_FAIL;
        else
          *status = TATS_DONE_SUCCESS;
      }
      break;
    default:
      *status = TATS_DONE_SUCCESS;
      break;
  }
}

void tactile_bridged_i2c_async_tick(const uint_fast8_t tactile_port)
{
  if (tactile_port != 2 && tactile_port != 3)
    return; // bogus
  const uint_fast8_t bridge_port = tactile_port - 2;
  tactile_bridged_i2c_status_t *status =
    &g_tactile_bridged_i2c_status[bridge_port];
  SPI_TypeDef *spi = NULL;
  GPIO_TypeDef *cs_gpio = NULL;
  uint32_t cs_pin_mask = 0;
  if (bridge_port == 0)
  {
    spi = SPI1;
    cs_gpio = GPIOA;
    cs_pin_mask = 1 << PORTA_BRIDGE0_CS;
  }
  else // bridge == 1
  {
    spi = SPI2;
    cs_gpio = GPIOB;
    cs_pin_mask = 1 << PORTB_BRIDGE1_CS;
  }

  switch (*status)
  {
    case TBPS_REQUEST_CS_LOW:
      if (SYSTIME - g_tactile_i2c_async_start_us[tactile_port] > 4)
      {
        spi->DR = g_tactile_i2c_async_address[tactile_port] & 0x1; // send CMD
        *status = TBPS_REQUEST_TX_CMD;
        g_tactile_i2c_async_start_us[tactile_port] = SYSTIME;
      }
      break;
    case TBPS_REQUEST_TX_CMD:
      if (SYSTIME - g_tactile_i2c_async_start_us[tactile_port] > 15)
      {
        spi->DR = g_tactile_i2c_async_data_len[tactile_port];
        *status = TBPS_REQUEST_TX_LEN;
        g_tactile_i2c_async_start_us[tactile_port] = SYSTIME;
      }
      break;
    case TBPS_REQUEST_TX_LEN:
      if (SYSTIME - g_tactile_i2c_async_start_us[tactile_port] > 15)
      {
        spi->DR = g_tactile_i2c_async_address[tactile_port]; // send addr
        *status = TBPS_REQUEST_TX_ADDR;
        g_tactile_i2c_async_start_us[tactile_port] = SYSTIME;
      }
      break;
    case TBPS_REQUEST_TX_ADDR:
      if (SYSTIME - g_tactile_i2c_async_start_us[tactile_port] > 15)
      {
        if ((g_tactile_i2c_async_address[tactile_port] & 0x1) || // it's a read
            g_tactile_i2c_async_data_len[tactile_port] == 0)     // or, no data
        {
          cs_gpio->BSRRL = cs_pin_mask; // de-assert CS
          *status = TBPS_WAIT_FOR_COMPLETION;
        }
        else
        {
          *status = TBPS_TX_DATA;
          g_tactile_i2c_async_data_txrx_idx[tactile_port] = 0;
          spi->DR = g_tactile_i2c_async_data[tactile_port][0];
        }
        g_tactile_i2c_async_start_us[tactile_port] = SYSTIME;
      }
      break;
    case TBPS_TX_DATA:
      if (SYSTIME - g_tactile_i2c_async_start_us[tactile_port] > 15)
      {
        if (g_tactile_i2c_async_data_txrx_idx[tactile_port] + 1 ==
            g_tactile_i2c_async_data_len[tactile_port])
        {
          cs_gpio->BSRRL = cs_pin_mask; // de-assert CS
          *status = TBPS_WAIT_FOR_COMPLETION;
        }
        else
        {
          g_tactile_i2c_async_data_txrx_idx[tactile_port]++;
          spi->DR = g_tactile_i2c_async_data[tactile_port][
                       g_tactile_i2c_async_data_txrx_idx[tactile_port]];
        }
        g_tactile_i2c_async_start_us[tactile_port] = SYSTIME;
      }
      break;
    case TBPS_WAIT_FOR_COMPLETION:
      {
        const uint32_t us_to_wait = 180 + 110 *
                                    g_tactile_i2c_async_data_len[tactile_port];
        if (SYSTIME - g_tactile_i2c_async_start_us[tactile_port] > us_to_wait)
        {
          if (g_tactile_i2c_async_address[tactile_port] & 0x1)
          {
            // it's a read. we need to harvest the data now.
            *status = TBPS_READ_CS_LOW;
            cs_gpio->BSRRH = cs_pin_mask;
            g_tactile_i2c_async_start_us[tactile_port] = SYSTIME;
          }
          else
            *status = TBPS_DONE_SUCCESS; // faster if we don't check...
        }
      }
      break;
    case TBPS_READ_CS_LOW:
      if (SYSTIME - g_tactile_i2c_async_start_us[tactile_port] > 4)
      {
        spi->DR = 0x06; // read buffer command
        spi->DR;
        g_tactile_i2c_async_start_us[tactile_port] = SYSTIME;
        *status = TBPS_READ_TXRX;
        g_tactile_i2c_async_data_txrx_idx[tactile_port] = 0;
      }
      break;
    case TBPS_READ_TXRX:
      if (SYSTIME - g_tactile_i2c_async_start_us[tactile_port] > 15)
      {
        volatile uint8_t dr = (uint8_t)spi->DR;
        if (g_tactile_i2c_async_data_txrx_idx[tactile_port] > 0)
        {
          g_tactile_i2c_async_data[tactile_port][
            g_tactile_i2c_async_data_txrx_idx[tactile_port]-1] = dr;
        }
        g_tactile_i2c_async_data_txrx_idx[tactile_port]++;
        if (g_tactile_i2c_async_data_txrx_idx[tactile_port] ==
            g_tactile_i2c_async_data_len[tactile_port] + 1)
        {
          cs_gpio->BSRRL = cs_pin_mask; // de-assert CS
          *status = TBPS_READ_TXRX_WAIT;
        }
        else
          spi->DR = 0x0;
        g_tactile_i2c_async_start_us[tactile_port] = SYSTIME;
      }
      break;
    case TBPS_READ_TXRX_WAIT:
      if (SYSTIME - g_tactile_i2c_async_start_us[tactile_port] > 30)
        *status = TBPS_DONE_SUCCESS;
      break;
    default:
      *status = TBPS_DONE_SUCCESS; // spin here after it's done
      break;
  }
}

void tactile_i2c_async_tick(const uint_fast8_t port)
{
  if (port >= NUM_TACTILE_PORTS)
    return; // let's not corrupt memory.
  if (port == 0 || port == 1) // on-chip i2c transceiver
    tactile_internal_i2c_async_tick(port);
  else
    tactile_bridged_i2c_async_tick(port);
}

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
  pin_set_output_level(GPIOA, PORTA_BRIDGE0_CS, 1);
  pin_set_output_level(GPIOA, PORTB_BRIDGE1_CS, 1);

  pin_set_alternate_function(GPIOA, PORTA_BRIDGE0_MISO, 5);
  pin_set_alternate_function(GPIOB, PORTB_BRIDGE0_MOSI, 5);
  pin_set_alternate_function(GPIOA, PORTA_BRIDGE0_SCLK, 5);
  pin_set_alternate_function(GPIOC, PORTC_BRIDGE1_MISO, 5);
  pin_set_alternate_function(GPIOC, PORTC_BRIDGE1_MOSI, 5);
  pin_set_alternate_function(GPIOD, PORTD_BRIDGE1_SCLK, 5);

  // spi1 is running from a 84 MHz pclk. set it up with
  // sclk = pclk/64 to stay within datasheet limits.
  SPI1->CR1 = SPI_CR1_BR_2 |
              SPI_CR1_BR_0 |
              SPI_CR1_MSTR |
              SPI_CR1_CPOL |
              SPI_CR1_CPHA |
              SPI_CR1_SSM  |
              SPI_CR1_SSI  |
              SPI_CR1_SPE;

  // bit rate = 42 mhz / 32 = 1.313 MHz
  SPI2->CR1 = SPI_CR1_BR_2 |
              SPI_CR1_MSTR |
              SPI_CR1_CPOL |
              SPI_CR1_CPHA |
              SPI_CR1_SSM  |
              SPI_CR1_SSI  |
              SPI_CR1_SPE;

  tactile_bridge_reset();
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 6; j++)
      printf("tactile bridge %d reg %d: 0x%02x\r\n",
             i, j, tactile_bridge_read_reg(i, j));
  // set the i2c bridges to run at 400 khz
  for (int i = 0; i < 2; i++)
    tactile_bridge_write_reg(i, 2, 5);

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

void tactile_poll(const uint_fast8_t port)
{
  if (port >= NUM_TACTILE_PORTS)
    return;

  // tell the MCU we want to broadcast to everybody
  if (tactile_i2c(port, BCAST_ENABLE_ADDR, NULL, 0) != I2C_SUCCESS)
    return;
  // tell everybody we want them to start their sampling process
  uint8_t msg[4] = { 0x12, 0x01, 0x00, 0x00};
  if (tactile_i2c(port, BAROM_ADDR, msg, 2) != I2C_SUCCESS)
    return;
  // disable everybody by reading one byte...
  if (tactile_i2c(port, BCAST_DISABLE_ADDR, msg, 1) != I2C_SUCCESS)
    return;

  for (volatile uint32_t i = 0; i < 10000; i++) { } // kill some time... SO BAD

  for (uint_fast8_t sensor_idx = 0;
       sensor_idx < g_tactile_sensors_per_port[port];
       sensor_idx++)
  {
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
  //printf("\r\n");
}

static uint_fast8_t tactile_sensor_addr(const uint_fast8_t tactile_port,
                                        const uint_fast8_t sensor_idx)
{
  if (tactile_port >= NUM_TACTILE_PORTS)
    return 0; // bogus
  if (tactile_port < NUM_FINGERS)
    return g_tactile_finger_addrs[sensor_idx];
  else
    return g_tactile_palm_addrs[sensor_idx];
}

static void tactile_bridge_write_reg(const uint8_t bridge_idx,
                                     const uint8_t reg_idx,
                                     const uint8_t reg_val)
{
  uint8_t txd[3] = {0x20, reg_idx, reg_val};
  tactile_bridge_spi_txrx(bridge_idx, 3, txd, NULL);
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
  for (volatile int i = 0; i < 10; i++) { } // la di dah...
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
  for (volatile int i = 0; i < 10; i++) { } // la di dah...
  cs_gpio->BSRRL = cs_pin_mask;
  for (volatile int i = 0; i < 10; i++) { } // la di dah...
}

tactile_i2c_result_t
tactile_bridge_wait_for_completion
  (const uint_fast8_t bridge_idx,
   const uint32_t wait_time)
{
  for (volatile int i = 0; i < wait_time; i++) { } // la di dah...
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

static tactile_i2c_result_t
tactile_bridge_i2c_write
  (const uint8_t bridge_idx,
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
  return tactile_bridge_wait_for_completion(bridge_idx, tx_len*3000);
}

static tactile_i2c_result_t
tactile_bridge_i2c_read
  (const uint8_t bridge_idx,
   const uint8_t i2c_addr,
   const uint8_t rx_len,
   uint8_t *rxd)
{
  uint8_t msg[3];
  msg[0] = 0x01; // read i2c command
  msg[1] = rx_len;
  msg[2] = i2c_addr; // will always performa a read... ignores the LSB
  tactile_bridge_spi_txrx(bridge_idx, 3, msg, NULL);
  if (tactile_bridge_wait_for_completion(bridge_idx, rx_len*2300) == I2C_FAIL)
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

void tactile_poll_nonblocking_tick(const uint8_t tactile_port)
{
  static uint_fast8_t errCount[NUM_TACTILE_PORTS] = {0};
  const uint_fast8_t tp = tactile_port; // save typing
  if (tp >= NUM_TACTILE_PORTS)
    return; // let's not corrupt memory.
  tactile_async_poll_state_t *tps = &tactile_poll_states[tp]; // save typing
  static uint_fast8_t active_sensor_idx[NUM_TACTILE_PORTS] = {0};
  int *i2c_status = NULL;
  if (tactile_port == 0 || tactile_port == 1)
    i2c_status = (int *)&g_tactile_internal_i2c_status[tactile_port];
  else if (tactile_port == 2 || tactile_port == 3)
    i2c_status = (int *)&g_tactile_bridged_i2c_status[tactile_port-2];
  else
    return; // shouldn't get here... but if somehow we do, it's time to bail

  if (*i2c_status == TACTILE_I2C_SUCCESS) {
    errCount[tactile_port] = 0;
    err_unset(ERR_TAC_0_PROBLEM + tactile_port);
  } else if (*i2c_status == TACTILE_I2C_FAIL) {
    if (errCount[tactile_port] > 100) {
      err_set(ERR_TAC_0_PROBLEM + tactile_port);
    } else {
      errCount[tactile_port]++;
    }
  }

  //const tactile_async_txrx_status *tats = &g_tactile_async_txrx_status[tp];
  const uint8_t sensor_count = (tp < NUM_FINGERS ?
                                SENSORS_PER_FINGER :
                                NUM_PALM_SENSORS);
  static uint32_t state_start_time_us[NUM_TACTILE_PORTS] = {0};
  switch (*tps)
  {
    case TPS_DONE: // initial state. kick things off.
      *tps = TPS_BCAST_ENABLE;
      tactile_i2c_async_start(tp, BCAST_ENABLE_ADDR, NULL, 0);
      break;
    case TPS_BCAST_ENABLE:
      tactile_i2c_async_tick(tp);
      if (*i2c_status == TACTILE_I2C_SUCCESS) {
        //*tps = TPS_DONE;
        uint8_t msg[2] = { 0x12, 0x01 };
        *tps = TPS_BCAST_START_SAMPLING;
        tactile_i2c_async_start(tp, BAROM_ADDR, msg, 2);
      } else if (*i2c_status == TACTILE_I2C_FAIL) {
        *tps = TPS_DONE;
      }
      break;
    case TPS_BCAST_START_SAMPLING:
      tactile_i2c_async_tick(tp);
      if (*i2c_status == TACTILE_I2C_SUCCESS) {
        *tps = TPS_BCAST_DISABLE;
        tactile_i2c_async_start(tp, BCAST_DISABLE_ADDR, NULL, 1);
      } else if (*i2c_status == TACTILE_I2C_FAIL) {
        *tps = TPS_DONE;
      }
      break;
    case TPS_BCAST_DISABLE:
      tactile_i2c_async_tick(tp);
      if (*i2c_status == TACTILE_I2C_SUCCESS) {
        *tps = TPS_SENSOR_SAMPLING;
        state_start_time_us[tp] = SYSTIME;
      } else if (*i2c_status == TACTILE_I2C_FAIL) {
        *tps = TPS_DONE;
      }
      break;
    case TPS_SENSOR_SAMPLING:
      // wait 3 ms
      if (SYSTIME - state_start_time_us[tp] > 3000) {
        active_sensor_idx[tp] = 0;
        const uint8_t sensor_addr = tactile_sensor_addr(tp, 0);
        *tps = TPS_SELECT_SENSOR;
        tactile_i2c_async_start(tp, sensor_addr, NULL, 0);
      }
      break;
    case TPS_SELECT_SENSOR:
      tactile_i2c_async_tick(tp);
      if (*i2c_status == TACTILE_I2C_SUCCESS) {
        uint8_t msg = 0;
        *tps = TPS_TX_READ_DATA_CMD;
        tactile_i2c_async_start(tp, BAROM_ADDR, &msg, 1);
      } else if (*i2c_status == TACTILE_I2C_FAIL) {
        *tps = TPS_DONE;
      }
      break;
    case TPS_TX_READ_DATA_CMD:
      tactile_i2c_async_tick(tp);
      if (*i2c_status == TACTILE_I2C_SUCCESS) {
        *tps = TPS_READ_DATA;
        tactile_i2c_async_start(tp, BAROM_ADDR | I2C_READ, NULL, 4);
      } else if (*i2c_status == TACTILE_I2C_FAIL) {
        *tps = TPS_DONE;
      }
      break;
    case TPS_READ_DATA:
      tactile_i2c_async_tick(tp);
      if (*i2c_status == TACTILE_I2C_SUCCESS) {
        uint8_t sensor_addr = tactile_sensor_addr(tp, active_sensor_idx[tp]);
        const uint8_t *p = g_tactile_i2c_async_data[tp];
        // Addition deals with small wraps
        const uint16_t pressure = 510 - (p[0]<200 ? ((uint16_t)p[0] + 255) : ((uint16_t)p[0]));
        const uint16_t temperature = ((uint16_t)p[2] << 2) | (p[3] >> 6);
        const uint_fast8_t state_sensor_idx = tp * SENSORS_PER_FINGER +
                                              active_sensor_idx[tp];
        g_state.tactile_pressures   [state_sensor_idx] = pressure;
        g_state.tactile_temperatures[state_sensor_idx] = temperature;
        //printf("port %d sensor 0x%02x pressure %06d  temperature %06d\r\n",
        //       tp, sensor_addr, pressure, temperature);
        *tps = TPS_DESELECT_SENSOR;
        tactile_i2c_async_start(tp, sensor_addr | I2C_READ, NULL, 1);
      } else if (*i2c_status == TACTILE_I2C_FAIL) {
        // move to the next sensor
        active_sensor_idx[tp]++;
        if (active_sensor_idx[tp] >= sensor_count) {
          *tps = TPS_DONE;
        } else {
          *tps = TPS_SELECT_SENSOR;
          const uint8_t sensor_addr = tactile_sensor_addr(tp,
                                           active_sensor_idx[tp]);
          tactile_i2c_async_start(tp, sensor_addr, NULL, 0);
        }
      }
      break;
    case TPS_DESELECT_SENSOR:
      tactile_i2c_async_tick(tp);
      if (*i2c_status == TACTILE_I2C_SUCCESS ||
          *i2c_status == TACTILE_I2C_FAIL) {
        active_sensor_idx[tp]++;
        if (active_sensor_idx[tp] >= sensor_count) {
          *tps = TPS_DONE;
        } else {
          *tps = TPS_SELECT_SENSOR;
          const uint8_t sensor_addr = tactile_sensor_addr(tp,
                                           active_sensor_idx[tp]);
          tactile_i2c_async_start(tp, sensor_addr, NULL, 0);
        }
      }
      break;
    default:
      *tps = TPS_DONE;
      break;
  }
}

