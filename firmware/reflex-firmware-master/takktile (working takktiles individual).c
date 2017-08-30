#include "takktile.h"

#define SLEEP_TIME 100

takktile_async_poll_state_t takktile_poll_states[NUM_TACTILE_PORTS] =
  { STATE_WAIT, STATE_WAIT, STATE_WAIT, STATE_WAIT}; // rmelo19

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
void takktile_bridge_reset();

static const uint8_t g_takktile_finger_addrs[SENSORS_PER_FINGER] =
{ 0x00, 0x02, 0x04, 0x06, 0x08, 0x10, 0x12, 0x14, 0x16 };

static const uint8_t g_takktile_sensors_per_port[NUM_TACTILE_PORTS] =
{ SENSORS_PER_FINGER, SENSORS_PER_FINGER, SENSORS_PER_FINGER, SENSORS_PER_FINGER};

#define TACTILE_I2C_SUCCESS 0xffffffff
#define TACTILE_I2C_FAIL    0xfffffffe

static volatile uint32_t g_takktile_i2c_async_start_us[NUM_TACTILE_PORTS] = {0};

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

void takktile_poll_nonblocking_tick(const uint8_t takktile_port)
{
  // static uint_fast8_t errCount[NUM_TACTILE_PORTS] = {0};
  static uint8_t sensorNumber[NUM_FINGERS] = {0, 0, 0};
  uint8_t sensorNumberAux;
  const uint_fast8_t tp = takktile_port; // save typing
  if (tp >= NUM_TACTILE_PORTS)
    return; // let's not corrupt memory.
  takktile_async_poll_state_t *tps = &takktile_poll_states[tp]; // save typing  

  // static uint_fast8_t active_sensor_idx[NUM_TACTILE_PORTS] = {0};
  // int *i2c_status = NULL;
  // if (takktile_port == 0 || takktile_port == 1)
  //   i2c_status = (int *)&g_takktile_internal_i2c_status[takktile_port];
  // else if (takktile_port == 2 || takktile_port == 3)
  //   i2c_status = (int *)&g_takktile_bridged_i2c_status[takktile_port-2];
  // else
  //   return; // shouldn't get here... but if somehow we do, it's time to bail

  // if (*i2c_status == TACTILE_I2C_SUCCESS)
  // {
  //   errCount[takktile_port] = 0;
  //   err_unset(ERR_TAC_0_PROBLEM + takktile_port);
  // }
  // else if (*i2c_status == TACTILE_I2C_FAIL)
  // {
  //   if (errCount[takktile_port] > 100)
  //   {
  //     err_set(ERR_TAC_0_PROBLEM + takktile_port);
  //   }
  //   else
  //   {
  //     errCount[takktile_port]++;
  //   }
  // }

  uint8_t takktileNumber;
  if (takktile_port == 2)
    takktileNumber = 1;
  else if (takktile_port == 0)
    takktileNumber = 0;
  else if (takktile_port == 1)
    takktileNumber = 2;
  else
    takktileNumber = 3;
  sensorNumberAux = sensorNumber[takktileNumber];

  //const takktile_async_txrx_status *tats = &g_takktile_async_txrx_status[tp];
  // const uint8_t sensor_count = (tp < NUM_FINGERS ? SENSORS_PER_FINGER : NUM_PALM_SENSORS);

  // static uint32_t state_start_time_us[NUM_TACTILE_PORTS] = {0};
  uint8_t result = 0;
  int delayTime = 100;
  switch (*tps)
  {
    case STATE_ENABLE_ALL_SENSORS:
        // printf("enableALLSensors %d\n", takktileNumber);
        udelay(delayTime); // CORRECT
        if (enableAllSensors(takktileNumber))
          *tps = STATE_START_CONVERSION;
      break;
    case STATE_START_CONVERSION:
        // printf("startConversionSequence\n");
        udelay(delayTime); // CORRECT
        if (startConversionSequence(takktileNumber))
          *tps = STATE_DISABLE_ALL_SENSORS;
      break;
    case STATE_DISABLE_ALL_SENSORS:
        // printf("disableAllSensors\n");
        udelay(delayTime); // CORRECT
        if (disableAllSensors(takktileNumber))
          *tps = STATE_ENABLE_SENSOR;
      break;
    case STATE_ENABLE_SENSOR:
        // printf("enableSensor\n");
        udelay(delayTime); // CORRECT
        if (enableSensor(takktileNumber, sensorNumberAux))
          *tps = STATE_SET_REGISTER;
      break;
    case STATE_SET_REGISTER:
        // printf("setRegister\n");
        udelay(delayTime); // CORRECT
        if (setRegister(takktileNumber))
          *tps = STATE_READ_VALUES;
      break;
    case STATE_READ_VALUES:
        // printf("readValues\n");
        udelay(delayTime); // CORRECT
        if (readValues(takktileNumber, sensorNumberAux))
          *tps = STATE_DISABLE_SENSOR;
      break;
    case STATE_DISABLE_SENSOR:
        // printf("disableSensor\n");
        udelay(delayTime); // CORRECT
        result = disableSensor(takktileNumber, sensorNumberAux);
        sensorNumber[takktileNumber]++;
        if (sensorNumberAux + 2 > SENSORS_PER_FINGER)
        {
          *tps = STATE_WAIT;
          sensorNumber[takktileNumber] = 0;
        }
        else if (result)
          *tps = STATE_ENABLE_SENSOR;
      break;
    case STATE_WAIT:    
      break;
    default:
      *tps = STATE_WAIT;
      break;
  }
}

uint8_t enableAllSensors(uint8_t takktileNumber)
{
  uint8_t result = 0;
  switch ((uint32_t) handPorts.takktile[takktileNumber])
  {
    case I2C1_BASE:
      result = writeBytesI2C(handPorts.takktile[takktileNumber], BCAST_ENABLE_ADDR, NULL, 0, 1);  // enable all sensors  
    break;
    case I2C3_BASE:
      result = writeBytesI2C(handPorts.takktile[takktileNumber], BCAST_ENABLE_ADDR, NULL, 0, 1);  // enable all sensors  
    break;
    case SPI1_BASE:
      result = writeBytesSPI(handPorts.takktile[takktileNumber], BCAST_ENABLE_ADDR, NULL, 0, 1);  // enable all sensors  
    break;
  }  
  udelay(SLEEP_TIME);
  return result;
}

uint8_t startConversionSequence(uint8_t takktileNumber)
{
  uint8_t result = 0;
  uint8_t data[2] = {0x12, 0x01};
  switch ((uint32_t) handPorts.takktile[takktileNumber])
  {
    case I2C1_BASE:
      result = writeBytesI2C(handPorts.takktile[takktileNumber], BAROM_ADDR, data, 2, 1);         // send Start Conversion Sequence
    break;
    case I2C3_BASE:
      result = writeBytesI2C(handPorts.takktile[takktileNumber], BAROM_ADDR, data, 2, 1);         // send Start Conversion Sequence
    break;
    case SPI1_BASE:
      result = writeBytesSPI(handPorts.takktile[takktileNumber], BAROM_ADDR, data, 2, 1);         // send Start Conversion Sequence
    break;
  }  
  udelay(SLEEP_TIME);
  return result;
}

uint8_t disableAllSensors(uint8_t takktileNumber)
{
  uint8_t result = 0;
  switch ((uint32_t) handPorts.takktile[takktileNumber])
  {
    uint8_t aux[1] = {0};
    case I2C1_BASE:
      result = readBytesI2C(handPorts.takktile[takktileNumber], BCAST_DISABLE_ADDR >> 1, 1, aux); // disable all sensors
    break;
    case I2C3_BASE:
      result = readBytesI2C(handPorts.takktile[takktileNumber], BCAST_DISABLE_ADDR >> 1, 1, aux); // disable all sensors
    break;
    case SPI1_BASE:
      result = readBytesSPI(handPorts.takktile[takktileNumber], BCAST_DISABLE_ADDR >> 1, 1, aux); // disable all sensors
    break;
  }  
  udelay(3000);                                     // wait 3ms
  return result;
}

uint8_t enableSensor(uint8_t takktileNumber, uint8_t sensorIndex)
{
  uint8_t result = 0;
  uint8_t addresses[9] = {0x00, 0x02, 0x04, 0x06, 0x08, 0x10, 0x12, 0x14, 0x16};
  switch ((uint32_t) handPorts.takktile[takktileNumber])
  {
    case I2C1_BASE:
      result = writeBytesI2C(handPorts.takktile[takktileNumber], addresses[sensorIndex], NULL, 0, 1); // enable sensor i
    break;
    case I2C3_BASE:
      result = writeBytesI2C(handPorts.takktile[takktileNumber], addresses[sensorIndex], NULL, 0, 1); // enable sensor i
    break;
    case SPI1_BASE:
      result = writeBytesSPI(handPorts.takktile[takktileNumber], addresses[sensorIndex], NULL, 0, 1); // enable sensor i
    break;
  }
  udelay(SLEEP_TIME);
  return result;
}

uint8_t setRegister(uint8_t takktileNumber)
{
  uint8_t result = 0;
  uint8_t msg[1] = {0x00};
  switch ((uint32_t) handPorts.takktile[takktileNumber])
  {
    case I2C1_BASE:
      result = writeBytesI2C(handPorts.takktile[takktileNumber], BAROM_ADDR, msg, 1, 1);       // choose register 0x00
    break;
    case I2C3_BASE:
      result = writeBytesI2C(handPorts.takktile[takktileNumber], BAROM_ADDR, msg, 1, 1);       // choose register 0x00
    break;
    case SPI1_BASE:
      result = writeBytesSPI(handPorts.takktile[takktileNumber], BAROM_ADDR, msg, 1, 1);       // choose register 0x00
    break;
  }
  udelay(SLEEP_TIME);
  return result;
}

uint8_t readValues(uint8_t takktileNumber, uint8_t sensorIndex)
{
  uint8_t result = 0;
  uint8_t tp;
  uint8_t index;
  uint8_t values[4] = {0, 0, 0, 0};
  switch ((uint32_t) handPorts.takktile[takktileNumber])
  {
    case I2C1_BASE:
      result = readBytesI2C(handPorts.takktile[takktileNumber], BAROM_ADDR >> 1, 4, values);      // read 4 bytes
    break;
    case I2C3_BASE:
      result = readBytesI2C(handPorts.takktile[takktileNumber], BAROM_ADDR >> 1, 4, values);      // read 4 bytes
    break;
    case SPI1_BASE:
      result = readBytesSPI(handPorts.takktile[takktileNumber], BAROM_ADDR >> 1, 4, values);      // read 4 bytes
    break;
  }
  if (takktileNumber == 2)
    tp = 1;
  else if (takktileNumber == 0)
    tp = 0;
  else if (takktileNumber == 1)
    tp = 2;
  else
    tp = 3;
  index = tp * SENSORS_PER_FINGER + sensorIndex;
  handState.takktile_pressures[index] = 510 - (values[0]<200 ? ((uint16_t)values[0] + 255) : ((uint16_t)values[0]));
  handState.takktile_temperatures[index] = ((uint16_t)values[2] << 2) | (values[3] >> 6);
  // printf("%3d ", handState.takktile_pressures[index]);
  udelay(SLEEP_TIME);
  return result;
}

uint8_t disableSensor(uint8_t takktileNumber, uint8_t sensorIndex)
{
  uint8_t result = 0;
  uint8_t addresses[9] = {0x00, 0x02, 0x04, 0x06, 0x08, 0x10, 0x12, 0x14, 0x16};
  uint8_t aux[1] = {0};
  switch ((uint32_t) handPorts.takktile[takktileNumber])
  {
    case I2C1_BASE:
      result = readBytesI2C(handPorts.takktile[takktileNumber], addresses[sensorIndex] >> 1, 1, aux);
    break;
    case I2C3_BASE:
      result = readBytesI2C(handPorts.takktile[takktileNumber], addresses[sensorIndex] >> 1, 1, aux);
    break;
    case SPI1_BASE:
      result = readBytesSPI(handPorts.takktile[takktileNumber], addresses[sensorIndex] >> 1, 1, aux);
    break;
  }
  udelay(SLEEP_TIME);
  return result;
}






