#include "takktile.h"

takktile_async_poll_state_t takktile_poll_states[NUM_TACTILE_PORTS] =
  { TPS_IDLE, TPS_IDLE, TPS_IDLE, TPS_IDLE}; // rmelo19

typedef enum { I2C_FAIL, I2C_SUCCESS } takktile_i2c_result_t;

static void takktile_bridge_spi_txrx(const uint8_t bridge_idx,
                                    const uint8_t txrx_len,
                                    const uint8_t *txd,
                                    uint8_t *rxd);
static uint8_t takktile_bridge_read_reg(const uint8_t bridge_idx,
                                       const uint8_t reg_idx);
static void takktile_bridge_write_reg(const uint8_t bridge_idx,
                                     const uint8_t reg_idx,
                                     const uint8_t reg_val);
static takktile_i2c_result_t takktile_bridge_i2c_write(const uint8_t bridge_idx,
                                     const uint8_t i2c_addr,
                                     const uint8_t tx_len,
                                     const uint8_t *txd);
static takktile_i2c_result_t
takktile_bridge_i2c_read(const uint8_t bridge_idx,
                        const uint8_t i2c_addr,
                        const uint8_t rx_len,
                        uint8_t *rxd);
void takktile_bridge_reset();

static const uint8_t g_takktile_finger_addrs[SENSORS_PER_FINGER] =
{ 0x00, 0x02, 0x04, 0x06, 0x08, 0x10, 0x12, 0x14, 0x16 };

static const uint8_t g_takktile_palm_addrs[NUM_PALM_SENSORS] =
{ 0x60, 0x62, 0x64, 0x66, 0x68, 0x70, 0x72, 0x74, 0x76, 0x78, 0x7a };

static const uint8_t g_takktile_sensors_per_port[NUM_TACTILE_PORTS] =
{ SENSORS_PER_FINGER, SENSORS_PER_FINGER, SENSORS_PER_FINGER, SENSORS_PER_FINGER};

takktile_i2c_result_t takktile_i2c(const uint8_t takktile_port,
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
} takktile_internal_i2c_status_t;

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
} takktile_bridged_i2c_status_t;

static uint8_t g_takktile_i2c_async_address      [NUM_TACTILE_PORTS] = {0};
static uint8_t g_takktile_i2c_async_data         [NUM_TACTILE_PORTS][256];
static uint8_t g_takktile_i2c_async_data_txrx_idx[NUM_TACTILE_PORTS] = {0};
static uint8_t g_takktile_i2c_async_data_len     [NUM_TACTILE_PORTS] = {0};
static volatile uint32_t g_takktile_i2c_async_start_us[NUM_TACTILE_PORTS] = {0};

static takktile_internal_i2c_status_t
g_takktile_internal_i2c_status[NUM_INTERNAL_I2C] = { TATS_START, TATS_START };

static bool g_takktile_i2c_async_address_fail[NUM_INTERNAL_I2C] = {false};

takktile_bridged_i2c_status_t
g_takktile_bridged_i2c_status[NUM_BRIDGED_I2C] =
{ TBPS_REQUEST_CS_LOW, TBPS_REQUEST_CS_LOW };

void takktile_i2c_async_start(const uint8_t port, const uint8_t address,
                             uint8_t *data, const uint8_t data_len)
{
  if (port >= NUM_TACTILE_PORTS - 1)
    return; // bogus

  g_takktile_i2c_async_address[port]  = address;
  g_takktile_i2c_async_data_len[port] = data_len;
  if (data)
    memcpy(g_takktile_i2c_async_data[port], data, data_len);
  else
    memset(g_takktile_i2c_async_data[port], 0, data_len);
  g_takktile_i2c_async_data_txrx_idx[port] = 0;

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
    // if (port != 0)
    // {
      i2c->CR1 |=  I2C_CR1_START;
      i2c->SR1 &= ~I2C_SR1_AF;
    // }

    g_takktile_internal_i2c_status[port] = TATS_START;
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
    g_takktile_bridged_i2c_status[bridge] = TBPS_REQUEST_CS_LOW;
  }
  g_takktile_i2c_async_start_us[port] = SYSTIME;
}

void takktile_internal_i2c_async_tick(const uint_fast8_t port)
{
  if (port >= NUM_INTERNAL_I2C)
    return; // bogus
  takktile_internal_i2c_status_t *status = &g_takktile_internal_i2c_status[port];
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
        i2c->DR = g_takktile_i2c_async_address[port];
        *status = TATS_ADDR;
      }
      break;
    case TATS_ADDR:
      if (i2c->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF))
      {
        bool address_fail = (i2c->SR1 & I2C_SR1_AF) ? true : false;
        g_takktile_i2c_async_address_fail[port] = address_fail;
        int no_payload = (0 == g_takktile_i2c_async_data_len[port]);
        if (no_payload)
          i2c->CR1 |= I2C_CR1_STOP; // this seemed needed... not sure why now.
        i2c->SR2; // un-stretch clock by reading here (?)
        if (!address_fail && !no_payload)
        {
          if (!(g_takktile_i2c_async_address[port] & 0x1))
          {
            // it's a write transaction
            g_takktile_i2c_async_data_txrx_idx[port] = 0;
            i2c->DR = g_takktile_i2c_async_data[port][0];
            *status = TATS_WRITE;
          }
          else
          {
            // it's a read transaction
            g_takktile_i2c_async_data_txrx_idx[port] = 0;
            if (g_takktile_i2c_async_data_len[port] == 1)
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
          g_takktile_i2c_async_data_txrx_idx[port]++;
          if (g_takktile_i2c_async_data_txrx_idx[port] >=
              g_takktile_i2c_async_data_len[port])
          {
            i2c->CR1 |= I2C_CR1_STOP;
            *status = TATS_STOP;
          }
          else
          {
            const uint8_t txrx_idx = g_takktile_i2c_async_data_txrx_idx[port];
            i2c->DR = g_takktile_i2c_async_data[port][txrx_idx];
          }
        }
      }
      break;
    case TATS_READ:
      if (i2c->SR1 & I2C_SR1_RXNE)
      {
        const uint8_t txrx_idx = g_takktile_i2c_async_data_txrx_idx[port];
        g_takktile_i2c_async_data[port][txrx_idx] = i2c->DR;
        g_takktile_i2c_async_data_txrx_idx[port]++;
        if (g_takktile_i2c_async_data_txrx_idx[port] >=
            g_takktile_i2c_async_data_len[port])
        {
          i2c->CR1 |= I2C_CR1_STOP;
          *status = TATS_STOP;
        }
        else
        {
          if (g_takktile_i2c_async_data_len[port] - 1 ==
              g_takktile_i2c_async_data_txrx_idx[port])
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
        g_takktile_i2c_async_start_us[port] = SYSTIME;
      }
      break;
    case TATS_STOP_WAIT:
      if (SYSTIME - g_takktile_i2c_async_start_us[port] > 10) // rmelo19
      {
        if (g_takktile_i2c_async_address_fail[port])
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

void takktile_bridged_i2c_async_tick(const uint_fast8_t takktile_port)
{
  if (takktile_port != 2 && takktile_port != 3)
    return; // bogus
  const uint_fast8_t bridge_port = takktile_port - 2;
  takktile_bridged_i2c_status_t *status =
    &g_takktile_bridged_i2c_status[bridge_port];
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
      if (SYSTIME - g_takktile_i2c_async_start_us[takktile_port] > 4)
      {
        spi->DR = g_takktile_i2c_async_address[takktile_port] & 0x1; // send CMD, read or write
        *status = TBPS_REQUEST_TX_CMD;
        g_takktile_i2c_async_start_us[takktile_port] = SYSTIME;
      }
      break;
    case TBPS_REQUEST_TX_CMD:
      if (SYSTIME - g_takktile_i2c_async_start_us[takktile_port] > 15)
      {
        spi->DR = g_takktile_i2c_async_data_len[takktile_port]; // send data len
        *status = TBPS_REQUEST_TX_LEN;
        g_takktile_i2c_async_start_us[takktile_port] = SYSTIME;
      }
      break;
    case TBPS_REQUEST_TX_LEN:
      if (SYSTIME - g_takktile_i2c_async_start_us[takktile_port] > 15)
      {
        spi->DR = g_takktile_i2c_async_address[takktile_port]; // send addr
        *status = TBPS_REQUEST_TX_ADDR;
        g_takktile_i2c_async_start_us[takktile_port] = SYSTIME;
      }
      break;
    case TBPS_REQUEST_TX_ADDR:
      if (SYSTIME - g_takktile_i2c_async_start_us[takktile_port] > 15)
      {
        if ((g_takktile_i2c_async_address[takktile_port] & 0x1) || // it's a read
            g_takktile_i2c_async_data_len[takktile_port] == 0)     // or, no data
        {
          cs_gpio->BSRRL = cs_pin_mask; // de-assert CS
          *status = TBPS_WAIT_FOR_COMPLETION;
        }
        else
        {
          *status = TBPS_TX_DATA;
          g_takktile_i2c_async_data_txrx_idx[takktile_port] = 0;
          spi->DR = g_takktile_i2c_async_data[takktile_port][0];
        }
        g_takktile_i2c_async_start_us[takktile_port] = SYSTIME;
      }
      break;
    case TBPS_TX_DATA:
      if (SYSTIME - g_takktile_i2c_async_start_us[takktile_port] > 15)
      {
        if (g_takktile_i2c_async_data_txrx_idx[takktile_port] + 1 ==
            g_takktile_i2c_async_data_len[takktile_port])
        {
          cs_gpio->BSRRL = cs_pin_mask; // de-assert CS
          *status = TBPS_WAIT_FOR_COMPLETION;
        }
        else
        {
          g_takktile_i2c_async_data_txrx_idx[takktile_port]++;
          spi->DR = g_takktile_i2c_async_data[takktile_port][
                       g_takktile_i2c_async_data_txrx_idx[takktile_port]];
        }
        g_takktile_i2c_async_start_us[takktile_port] = SYSTIME;
      }
      break;
    case TBPS_WAIT_FOR_COMPLETION:
      {
        const uint32_t us_to_wait = 180 + 110 *
                                    g_takktile_i2c_async_data_len[takktile_port];
        if (SYSTIME - g_takktile_i2c_async_start_us[takktile_port] > us_to_wait)
        {
          if (g_takktile_i2c_async_address[takktile_port] & 0x1)
          {
            // it's a read. we need to harvest the data now.
            *status = TBPS_READ_CS_LOW;
            cs_gpio->BSRRH = cs_pin_mask;
            g_takktile_i2c_async_start_us[takktile_port] = SYSTIME;
          }
          else
            *status = TBPS_DONE_SUCCESS; // faster if we don't check...
        }
      }
      break;
    case TBPS_READ_CS_LOW:
      if (SYSTIME - g_takktile_i2c_async_start_us[takktile_port] > 4)
      {
        spi->DR = 0x06; // read buffer command
        spi->DR;
        g_takktile_i2c_async_start_us[takktile_port] = SYSTIME;
        *status = TBPS_READ_TXRX;
        g_takktile_i2c_async_data_txrx_idx[takktile_port] = 0;
      }
      break;
    case TBPS_READ_TXRX:
      if (SYSTIME - g_takktile_i2c_async_start_us[takktile_port] > 15)
      {
        volatile uint8_t dr = (uint8_t)spi->DR;
        if (g_takktile_i2c_async_data_txrx_idx[takktile_port] > 0)
        {
          g_takktile_i2c_async_data[takktile_port][
            g_takktile_i2c_async_data_txrx_idx[takktile_port]-1] = dr;
        } 
        g_takktile_i2c_async_data_txrx_idx[takktile_port]++;
        if (g_takktile_i2c_async_data_txrx_idx[takktile_port] ==
            g_takktile_i2c_async_data_len[takktile_port] + 1)
        {
          cs_gpio->BSRRL = cs_pin_mask; // de-assert CS
          *status = TBPS_READ_TXRX_WAIT;
        }
        else
          spi->DR = 0x0;
        g_takktile_i2c_async_start_us[takktile_port] = SYSTIME;
      }
      break;
    case TBPS_READ_TXRX_WAIT:
      if (SYSTIME - g_takktile_i2c_async_start_us[takktile_port] > 30)
        *status = TBPS_DONE_SUCCESS;
      break;
    default:
      *status = TBPS_DONE_SUCCESS; // spin here after it's done
      break;
  }
}

void takktile_i2c_async_tick(const uint_fast8_t port)
{
  if (port >= NUM_TACTILE_PORTS)
    return; // let's not corrupt memory.
  if (port == 0 || port == 1) // on-chip i2c transceiver
    takktile_internal_i2c_async_tick(port);
  else
    takktile_bridged_i2c_async_tick(port);
}

//////////////////////////////////////////////////////////////////////////////

void takktileInit()
{
  takktile_bridge_reset();
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 6; j++)
      printf("takktile bridge %d reg %d: 0x%02x\r\n", i, j, takktile_bridge_read_reg(i, j));

  // set the i2c bridges to run at 400 khz
  for (int i = 0; i < 2; i++)
    takktile_bridge_write_reg(i, 2, 5);

  /*
  for (int port = 0; port < NUM_TACTILE_PORTS; port++)
  {
    for (int sensor = 0; sensor < g_takktile_sensors_per_port[port]; sensor++)
    {
      const uint8_t mcu_addr = port < NUM_FINGERS ?
                               g_takktile_finger_addrs[sensor] :
                               g_takktile_palm_addrs[sensor];
      takktile_i2c_result_t result =
        takktile_i2c(port, mcu_addr, NULL, 0) == I2C_SUCCESS &&
        takktile_i2c(port, BAROM_ADDR, NULL, 0) == I2C_SUCCESS;
      printf("%s: port %d sensor %d at addr 0x%02x\r\n",
             result == I2C_SUCCESS ? "SUCCESS" : "FAIL",
             port, sensor, mcu_addr);
    }
  }
  */
  printf("done with takktile_init()\r\n");
}

void takktile_bridge_reset()
{
  // the two SPI-I2C bridges have their RESETs tied to a common MCU pin,
  // so this function will reset both of them
  printf("takktile_bridge_reset()\r\n");
  GPIOC->BSRRH = 1 << PORTC_I2C_BRIDGE_RESET;
  for (volatile int i = 0; i < 10000; i++) { } // assert RESET
  GPIOC->BSRRL = 1 << PORTC_I2C_BRIDGE_RESET;
  for (volatile int i = 0; i < 10000; i++) { } // then let them boot up
}

takktile_i2c_result_t takktile_i2c(uint8_t port,
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
      return takktile_bridge_i2c_read(bridge_idx, address, data_len, data);
    else
      return takktile_bridge_i2c_write(bridge_idx, address, data_len, data);
  }
  else
    return I2C_FAIL;
}

static uint_fast8_t takktile_sensor_addr(const uint_fast8_t takktile_port,
                                        const uint_fast8_t sensor_idx)
{
  if (takktile_port >= NUM_TACTILE_PORTS)
    return 0; // bogus
  if (takktile_port < NUM_FINGERS)
    return g_takktile_finger_addrs[sensor_idx];
  else
    return g_takktile_palm_addrs[sensor_idx];
}

static void takktile_bridge_write_reg(const uint8_t bridge_idx,
                                     const uint8_t reg_idx,
                                     const uint8_t reg_val)
{
  uint8_t txd[3] = {0x20, reg_idx, reg_val};
  takktile_bridge_spi_txrx(bridge_idx, 3, txd, NULL);
}

static uint8_t takktile_bridge_read_reg(const uint8_t bridge_idx,
                                       const uint8_t reg_idx)
{
  uint8_t rxd[3] = {0};
  uint8_t txd[3] = {0x21, reg_idx, 0};
  takktile_bridge_spi_txrx(bridge_idx, 3, txd, rxd);
  const uint8_t reg_val = rxd[2];
  //printf("spi reg 0x%02x = 0x%02x\r\n", reg_idx, reg_val);
  return reg_val;
}

static void takktile_bridge_spi_txrx(const uint8_t bridge_idx,
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

takktile_i2c_result_t
takktile_bridge_wait_for_completion
  (const uint_fast8_t bridge_idx,
   const uint32_t wait_time)
{
  for (volatile int i = 0; i < wait_time; i++) { } // la di dah...
  uint8_t bridge_state = 0xf3;
  int wait_count = 0;
  while (bridge_state == 0xf3)
  {
    for (volatile int i = 0; i < 1000; i++) { } // la di dah...
    bridge_state = takktile_bridge_read_reg(bridge_idx, 0x4);
    if (++wait_count > 1000)
    {
      // a SPI-I2C bridge locked up. try to reset them.
      takktile_bridge_reset();
      return I2C_FAIL;
    }
    //printf("bridge_state = 0x%02x\r\n", bridge_state);
  }
  if (bridge_state == 0xf0)
    return I2C_SUCCESS;
  else
    return I2C_FAIL;
}

static takktile_i2c_result_t
takktile_bridge_i2c_write
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
  takktile_bridge_spi_txrx(bridge_idx, trimmed_tx_len + 3, msg, NULL);
  return takktile_bridge_wait_for_completion(bridge_idx, tx_len*3000);
}

static takktile_i2c_result_t
takktile_bridge_i2c_read
  (const uint8_t bridge_idx,
   const uint8_t i2c_addr,
   const uint8_t rx_len,
   uint8_t *rxd)
{
  uint8_t msg[3];
  msg[0] = 0x01; // read i2c command
  msg[1] = rx_len;
  msg[2] = i2c_addr; // will always performa a read... ignores the LSB
  takktile_bridge_spi_txrx(bridge_idx, 3, msg, NULL);
  if (takktile_bridge_wait_for_completion(bridge_idx, rx_len*2300) == I2C_FAIL)
    return I2C_FAIL;
  uint8_t read_msg[rx_len+1], rx_msg[rx_len+1];
  read_msg[0] = 0x06; // read buffer command
  for (int i = 1; i < rx_len+1; i++)
    read_msg[i] = 0;
  takktile_bridge_spi_txrx(bridge_idx, rx_len+1, read_msg, rx_msg);
  for (int i = 1; i < rx_len+1; i++)
    rxd[i-1] = rx_msg[i];
  for (volatile int i = 0; i < 1000; i++) { } // la di dah...
  uint8_t bridge_state = takktile_bridge_read_reg(bridge_idx, 0x4);
  return (bridge_state == 0xf0 ? I2C_SUCCESS : I2C_FAIL);
}

void takktile_poll_nonblocking_tick(const uint8_t takktile_port)
{
  static uint_fast8_t errCount[NUM_TACTILE_PORTS] = {0};
  const uint_fast8_t tp = takktile_port; // save typing
  if (tp >= NUM_TACTILE_PORTS)
    return; // let's not corrupt memory.
  takktile_async_poll_state_t *tps = &takktile_poll_states[tp]; // save typing  

  static uint_fast8_t active_sensor_idx[NUM_TACTILE_PORTS] = {0};
  int *i2c_status = NULL;
  if (takktile_port == 0 || takktile_port == 1)
    i2c_status = (int *)&g_takktile_internal_i2c_status[takktile_port];
  else if (takktile_port == 2 || takktile_port == 3)
    i2c_status = (int *)&g_takktile_bridged_i2c_status[takktile_port-2];
  else
    return; // shouldn't get here... but if somehow we do, it's time to bail

  if (*i2c_status == TACTILE_I2C_SUCCESS) {
    errCount[takktile_port] = 0;
    err_unset(ERR_TAC_0_PROBLEM + takktile_port);
  } else if (*i2c_status == TACTILE_I2C_FAIL) {
    if (errCount[takktile_port] > 100) {
      err_set(ERR_TAC_0_PROBLEM + takktile_port);
    } else {
      errCount[takktile_port]++;
    }
  }

  //const takktile_async_txrx_status *tats = &g_takktile_async_txrx_status[tp];
  const uint8_t sensor_count = (tp < NUM_FINGERS ? SENSORS_PER_FINGER : NUM_PALM_SENSORS);

  static uint32_t state_start_time_us[NUM_TACTILE_PORTS] = {0};
  switch (*tps)
  {
    case TPS_DONE: // initial state. kick things off.
      if (takktile_port == 2)
      {
        uint_fast8_t index;
        uint8_t sleepTime = 100;
        uint8_t data[2] = {0x12, 0x01};
        uint8_t msg[1] = {0x00};
        uint8_t aux[1] = {0x00};
        writeBytesSPI(SPI1, BCAST_ENABLE_ADDR, NULL, 0, 1);  // enable all sensors    
        udelay(sleepTime);
        writeBytesSPI(SPI1, BAROM_ADDR, data, 2, 1);         // send Start Conversion Sequence
        udelay(sleepTime);
        readBytesSPI(SPI1, BCAST_DISABLE_ADDR >> 1, 1, aux); // disable all sensors
        udelay(3000);                                     // wait 3ms

        uint8_t values[4] = {0, 0, 0, 0};
        uint8_t addresses[9] = {0x00, 0x02, 0x04, 0x06, 0x08, 0x10, 0x12, 0x14, 0x16};


        // printf("Pressures 1: ");

        // for (int i = 0; i < 9; i++)
        // {
        //   index = 0 * SENSORS_PER_FINGER + i;
        //   printf("%3d ", handState.takktile_pressures[index]);
        // }

        // printf("\n");
       
        printf("Pressures 2: ");

        for (int i = 0; i < 9; i++)
        {
          
          writeBytesSPI(SPI1, addresses[i], NULL, 0, 1); // enable sensor i
          udelay(sleepTime);

          writeBytesSPI(SPI1, BAROM_ADDR, msg, 1, 1);       // choose register 0x00
          udelay(sleepTime);

          readBytesSPI(SPI1, BAROM_ADDR >> 1, 4, values);      // read 4 bytes
          udelay(sleepTime);

          index = tp * SENSORS_PER_FINGER + i;
          handState.takktile_pressures[index] = 510 - (values[0]<200 ? ((uint16_t)values[0] + 255) : ((uint16_t)values[0]));
          handState.takktile_temperatures[index] = ((uint16_t)values[2] << 2) | (values[3] >> 6);
          printf("%3d ", handState.takktile_pressures[index]);

          readBytesSPI(SPI1, addresses[i] >> 1, 1, aux);
          udelay(sleepTime);
        }

        printf("\n");

        // printf("Pressures 3: ");

        // for (int i = 0; i < 9; i++)
        // {
        //   index = 1 * SENSORS_PER_FINGER + i;
        //   printf("%3d ", handState.takktile_pressures[index]);
        // }

        // printf("\n");
          // uint_fast8_t index;

          // uint8_t data[2] = {0x12, 0x01};
          // writeBytesSPI(SPI1, BCAST_ENABLE_ADDR, NULL, 0, 1);  // enable all sensors    
          // // udelay(SLEEP_TIME);
          // writeBytesSPI(SPI1, BAROM_ADDR, data, 2, 1);         // send Start Conversion Sequence
          // // udelay(SLEEP_TIME); // test
          // // writeBytesSPI(SPI1, BCAST_DISABLE_ADDR, NULL, 0, 1); // disable all sensors
          // readBytesSPI(SPI1, BCAST_DISABLE_ADDR>>1, 0, NULL);
          // udelay(3000);                                     // wait 3ms
          // for (int i = 7; i < 8; i++)
          // {
          //   volatile uint8_t values[4] = {0, 0, 0, 0};
          //   writeBytesSPI(SPI1, takktile_sensor_addr(tp, i), NULL, 0, 1); // enable sensor i
          //   // udelay(SLEEP_TIME); // test
          //   uint8_t msg[1] = {0};
          //   writeBytesSPI(SPI1, BAROM_ADDR, msg, 1, 1);       // choose register 0x00
          //   // udelay(SLEEP_TIME); // test
          //   readBytesSPI(SPI1, BAROM_ADDR >> 1, 1, values);      // read 4 bytes
          //   // udelay(SLEEP_TIME); // test
          //   index = tp * SENSORS_PER_FINGER + i;
          //   handState.takktile_pressures   [index] = 510 - (values[0]<200 ? ((uint16_t)values[0] + 255) : ((uint16_t)values[0]));
          //   handState.takktile_temperatures[index] = ((uint16_t)values[2] << 2) | (values[3] >> 6);;
          //   // writeBytesSPI(SPI1, takktile_sensor_addr(tp, i) + 1, NULL, 0, 1); // disable sensor i
          //   readBytesSPI(SPI1, takktile_sensor_addr(tp, i)>>1, 0, NULL);
          //   // udelay(SLEEP_TIME); // test
          // }
      }
      else if (takktile_port == 3)
      {

      }
      else
      {
         printf("Pressures %d: ", takktile_port);

        for (int i = 0; i < 9; i++)
        {
          const uint_fast8_t state_sensor_idx = ((uint8_t) tp) * SENSORS_PER_FINGER + i;
          printf("%3d ", handState.takktile_pressures[state_sensor_idx]);
        }

        printf("\n");

        *tps = TPS_BCAST_ENABLE;
        takktile_i2c_async_start(tp, BCAST_ENABLE_ADDR, NULL, 0);  
      }
      break;
    case TPS_BCAST_ENABLE:
      takktile_i2c_async_tick(tp);
      if (*i2c_status == TACTILE_I2C_SUCCESS) {
        //*tps = TPS_DONE;
        uint8_t msg[2] = { 0x12, 0x01 };
        *tps = TPS_BCAST_START_SAMPLING;
        takktile_i2c_async_start(tp, BAROM_ADDR, msg, 2);
      } else if (*i2c_status == TACTILE_I2C_FAIL) {
        *tps = TPS_DONE;
      }
      break;
    case TPS_BCAST_START_SAMPLING:
      takktile_i2c_async_tick(tp);
      if (*i2c_status == TACTILE_I2C_SUCCESS) {
        *tps = TPS_BCAST_DISABLE;
        takktile_i2c_async_start(tp, BCAST_DISABLE_ADDR, NULL, 1);
      } else if (*i2c_status == TACTILE_I2C_FAIL) {
        *tps = TPS_DONE;
      }
      break;
    case TPS_BCAST_DISABLE:
      takktile_i2c_async_tick(tp);
      if (*i2c_status == TACTILE_I2C_SUCCESS) {
        *tps = TPS_SENSOR_SAMPLING;
        state_start_time_us[tp] = SYSTIME;
      } else if (*i2c_status == TACTILE_I2C_FAIL) {
        *tps = TPS_DONE;
      }
      break;
    case TPS_SENSOR_SAMPLING:
      // wait 3 ms
      if (SYSTIME - state_start_time_us[tp] > 3000) { // CORRECT
        active_sensor_idx[tp] = 0;
        const uint8_t sensor_addr = takktile_sensor_addr(tp, 0);
        *tps = TPS_SELECT_SENSOR;
        takktile_i2c_async_start(tp, sensor_addr, NULL, 0);
      }
      break;
    case TPS_SELECT_SENSOR:
      takktile_i2c_async_tick(tp);
      if (*i2c_status == TACTILE_I2C_SUCCESS) {
        uint8_t msg = 0;
        *tps = TPS_TX_READ_DATA_CMD;
        takktile_i2c_async_start(tp, BAROM_ADDR, &msg, 1);
      } else if (*i2c_status == TACTILE_I2C_FAIL) {
        *tps = TPS_DONE;
      }
      break;
    case TPS_TX_READ_DATA_CMD:
      takktile_i2c_async_tick(tp);
      if (*i2c_status == TACTILE_I2C_SUCCESS) {
        *tps = TPS_READ_DATA;
        takktile_i2c_async_start(tp, BAROM_ADDR | I2C_READ, NULL, 4);
      } else if (*i2c_status == TACTILE_I2C_FAIL) {
        *tps = TPS_DONE;
      }
      break;
    case TPS_READ_DATA:
      takktile_i2c_async_tick(tp);
      if (*i2c_status == TACTILE_I2C_SUCCESS) {
        uint8_t sensor_addr = takktile_sensor_addr(tp, active_sensor_idx[tp]);
        const uint8_t *p = g_takktile_i2c_async_data[tp];
        // Addition deals with small wraps
        const uint16_t pressure = 510 - (p[0]<200 ? ((uint16_t)p[0] + 255) : ((uint16_t)p[0]));
        const uint16_t temperature = ((uint16_t)p[2] << 2) | (p[3] >> 6);
        const uint_fast8_t state_sensor_idx = tp * SENSORS_PER_FINGER + active_sensor_idx[tp];
        handState.takktile_pressures   [state_sensor_idx] = pressure;
        handState.takktile_temperatures[state_sensor_idx] = temperature;

        //printf("port %d sensor 0x%02x pressure %06d  temperature %06d\r\n",
        //       tp, sensor_addr, pressure, temperature);
        *tps = TPS_DESELECT_SENSOR;
        takktile_i2c_async_start(tp, sensor_addr | I2C_READ, NULL, 1);
      } else if (*i2c_status == TACTILE_I2C_FAIL) {
        // move to the next sensor
        active_sensor_idx[tp]++;
        if (active_sensor_idx[tp] >= sensor_count) {
          *tps = TPS_DONE;
        } else {
          *tps = TPS_SELECT_SENSOR;
          const uint8_t sensor_addr = takktile_sensor_addr(tp,
                                           active_sensor_idx[tp]);
          takktile_i2c_async_start(tp, sensor_addr, NULL, 0);
        }
      }
      break;
    case TPS_DESELECT_SENSOR:
      takktile_i2c_async_tick(tp);
      if (*i2c_status == TACTILE_I2C_SUCCESS ||
          *i2c_status == TACTILE_I2C_FAIL) {
        active_sensor_idx[tp]++;
        if (active_sensor_idx[tp] >= sensor_count) {
          *tps = TPS_DONE;
        } else {
          *tps = TPS_SELECT_SENSOR;
          const uint8_t sensor_addr = takktile_sensor_addr(tp,
                                           active_sensor_idx[tp]);
          takktile_i2c_async_start(tp, sensor_addr, NULL, 0);
        }
      }
      break;
    default:
      *tps = TPS_DONE;
      break;
  }
}

