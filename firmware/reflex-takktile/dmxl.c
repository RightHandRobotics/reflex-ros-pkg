#include "dmxl.h"
#include "stm32f4xx.h"
#include <stdio.h>
#include "state.h"
#include "systime.h"
#include "delay.h"

#define PORTA_DMXL_BUF_EN 10

typedef enum
{
  DMXL_CS_IDLE        = 0,
  DMXL_CS_TX          = 1,
  DMXL_CS_POLL_STATE  = 2,
  DMXL_CS_POLL_DEBRIS = 3,
  DMXL_CS_PING        = 4
} dmxl_comms_state_t;

typedef enum
{
  DMXL_PS_PREAMBLE_0 = 0,
  DMXL_PS_PREAMBLE_1 = 1,
  DMXL_PS_ID         = 2,
  DMXL_PS_LENGTH     = 3,
  DMXL_PS_ERROR      = 4,
  DMXL_PS_PARAMETER  = 5,
  DMXL_PS_CHECKSUM   = 6
} dmxl_parser_state_t;

typedef struct
{
  GPIO_TypeDef *tx_gpio, *rx_gpio;
  uint8_t tx_pin, rx_pin, af;
  USART_TypeDef *uart;
  dmxl_comms_state_t  comms_state;
  dmxl_parser_state_t parser_state;
  uint8_t rx_pkt_len, rx_pkt_writepos, rx_checksum;
} dmxl_port_t;

static dmxl_port_t g_dmxl_ports[NUM_DMXL] =
{
  { GPIOD, GPIOD,  8,  9, 7, USART3, DMXL_CS_IDLE, DMXL_PS_PREAMBLE_0, 0,0,0 },
  { GPIOC, GPIOC, 10, 11, 8, UART4 , DMXL_CS_IDLE, DMXL_PS_PREAMBLE_0, 0,0,0 },
  { GPIOC, GPIOC,  6,  7, 8, USART6, DMXL_CS_IDLE, DMXL_PS_PREAMBLE_0, 0,0,0 },
  { GPIOC, GPIOD, 12,  2, 8, UART5 , DMXL_CS_IDLE, DMXL_PS_PREAMBLE_0, 0,0,0 }
};

#define DMXL_RING_LEN 256
static volatile uint8_t  g_dmxl_ring[NUM_DMXL][DMXL_RING_LEN];
static volatile uint16_t g_dmxl_ring_rpos[NUM_DMXL] = {0};
static volatile uint16_t g_dmxl_ring_wpos[NUM_DMXL] = {0};
static volatile uint8_t  g_dmxl_rx_pkt[NUM_DMXL][256];
static volatile dmxl_control_mode_t dmxl_control_mode = DMXL_CM_IDLE;
static volatile uint8_t divider_is_sent[NUM_DMXL] = {0};

dmxl_async_poll_state_t dmxl_poll_states[NUM_DMXL] =
  { DPS_DONE, DPS_DONE, DPS_DONE, DPS_DONE };

static void dmxl_read_data(const uint8_t port_idx, const uint8_t dmxl_id,
                           const uint8_t data_len, const uint8_t start_addr);
static void dmxl_write_data(const uint8_t port_idx, const uint8_t dmxl_id,
                            const uint8_t data_len, const uint8_t start_addr,
                            const uint8_t *data);
static uint8_t dmxl_send_ping(const uint8_t port_idx, const uint8_t dmxl_id);

// put in ramfunc sector ?
static inline void dmxl_push_byte(const uint8_t dmxl_port, const uint8_t byte)
{
  if (dmxl_port >= NUM_DMXL)
    return;
  g_dmxl_ring[dmxl_port][g_dmxl_ring_wpos[dmxl_port]] = byte;
  if (++g_dmxl_ring_wpos[dmxl_port] >= DMXL_RING_LEN)
    g_dmxl_ring_wpos[dmxl_port] = 0;
}

void usart3_vector()
{
  volatile uint8_t __attribute__((unused)) sr = USART3->SR; // clear overruns
  dmxl_push_byte(0, USART3->DR);
}

void uart4_vector()
{
  volatile uint8_t __attribute__((unused)) sr = UART4->SR; // clear overruns
  dmxl_push_byte(1, UART4->DR);
}

void uart5_vector()
{
  volatile uint8_t __attribute__((unused)) sr = UART5->SR; // clear overruns
  dmxl_push_byte(3, UART5->DR);
}

void usart6_vector()
{
  volatile uint8_t __attribute__((unused)) sr = USART6->SR; // clear overruns
  dmxl_push_byte(2, USART6->DR);
}

void dmxl_set_usart_baud(const uint_fast8_t dmxl_idx, int baud)
{
  USART_TypeDef *u = g_dmxl_ports[dmxl_idx].uart;
  if (baud == 57600)
  {
    if (u == USART6)
      u->BRR = (((uint16_t)91) << 4) | 2;
    else
      u->BRR = (((uint16_t)45) << 4) | 9;
  }
  else if (baud == 250000)
  {
    if (u == USART6)
      u->BRR = (((uint16_t)21) << 4);
    else
      u->BRR = (((uint16_t)10) << 4) | 8;
  }
  else if (baud == 1000000)
  {
    if (u == USART6)
      u->BRR = (((uint16_t)5) << 4) |  4;
    else
      u->BRR = (((uint16_t)2) << 4) | 10;
  }
  else
    printf("ahhh unhandled baud rate: %d\r\n", baud);
}

void dmxl_init()
{
  printf("dmxl_init()\r\n");
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                  RCC_AHB1ENR_GPIOCEN |
                  RCC_AHB1ENR_GPIODEN;
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN |
                  RCC_APB1ENR_UART4EN  |
                  RCC_APB1ENR_UART5EN;
  RCC->APB2ENR |= RCC_APB2ENR_USART6EN;

  for (int i = 0; i < NUM_DMXL; i++)
  {
    dmxl_port_t *dp = &g_dmxl_ports[i];
    dp->tx_gpio->MODER |= (uint32_t)2 << (dp->tx_pin * 2); // set as AF
    dp->rx_gpio->MODER |= (uint32_t)2 << (dp->rx_pin * 2); // set as AF

    if (dp->tx_pin >= 8)
      dp->tx_gpio->AFR[1] |= (uint32_t)dp->af << ((dp->tx_pin - 8) * 4);
    else
      dp->tx_gpio->AFR[0] |= (uint32_t)dp->af << (dp->tx_pin * 4);

    if (dp->rx_pin >= 8)
      dp->rx_gpio->AFR[1] |= (uint32_t)dp->af << ((dp->rx_pin - 8) * 4);
    else
      dp->rx_gpio->AFR[0] |= (uint32_t)dp->af << (dp->rx_pin * 4);

    USART_TypeDef *u = dp->uart;
    u->CR1 &= ~USART_CR1_UE; // make sure it's off while we configure it
    u->CR1 |=  USART_CR1_RE; // don't enable the transmitter yet

    dmxl_set_usart_baud(i, 250000);
    /*
    if (u == USART6) // running on APB2 = 84 MHz
      u->BRR = (((uint16_t)5) << 4) | 4; // 5.25 mantissa = 5, fraction =  4
    else // running on APB1 = 42 MHz
      u->BRR = (((uint16_t)2) << 4) | 10;// 2.625 mantissa = 2, fraction = 10
    */
    u->CR1 |=  USART_CR1_UE | USART_CR1_RXNEIE;

    delay_us(10);
    dmxl_set_res_divider(i, DMXL_DEFAULT_ID, 4);
    delay_us(10);
    dmxl_set_multiturn_offset(i, DMXL_DEFAULT_ID, 13000);  // Places motor close enough to middle of 0-28672 range
  }
  NVIC_SetPriority(USART3_IRQn, 2);
  NVIC_SetPriority(UART4_IRQn, 2);
  NVIC_SetPriority(UART5_IRQn, 2);
  NVIC_SetPriority(USART6_IRQn, 2);

  NVIC_EnableIRQ(USART3_IRQn);
  NVIC_EnableIRQ(UART4_IRQn);
  NVIC_EnableIRQ(UART5_IRQn);
  NVIC_EnableIRQ(USART6_IRQn);

  //GPIOD->MODER |= (1 << (8*2)); // | (1 << 9);
  // turn on the 3.3v <-> 5v translator chip now
  GPIOA->MODER |= 1 << (PORTA_DMXL_BUF_EN * 2);
  GPIOA->BSRRL |= 1 << PORTA_DMXL_BUF_EN;
}

void dmxl_set_status_return_levels()
{
  // see what the Status Return Level is. If it's not 1, set it to one
  for (int i = 0; i < NUM_DMXL; i++)
  {
    g_dmxl_ports[i].comms_state = DMXL_CS_POLL_DEBRIS;
    dmxl_read_data(i, DMXL_DEFAULT_ID, 1, 0x10);
    volatile uint32_t t_start = SYSTIME;
    uint8_t status_return_level = 0xff;
    while (SYSTIME - t_start < 10000)
    {
      dmxl_process_rings();
      if (g_dmxl_ports[i].comms_state != DMXL_CS_POLL_DEBRIS)
      {
        status_return_level = g_dmxl_rx_pkt[i][0];
        printf("dmxl %d status return level = %d\n", i, status_return_level);
        break;
      }
    }
    if (status_return_level == 0xff)
      printf("couldn't poll status return level for dmxl %d\r\n", i);
    else if (status_return_level != 1)
    {
      // set it to 2
      uint8_t level = 1; // only respond to READ
      dmxl_write_data(i, DMXL_DEFAULT_ID, 1, 0x10, &level);
    }
  }
}

void dmxl_set_baud_rates()
{
  for (int i = 0; i < NUM_DMXL; i++)
  {
    dmxl_set_usart_baud(i, 250000);
    g_dmxl_ports[i].comms_state = DMXL_CS_PING;
    dmxl_send_ping(i, DMXL_DEFAULT_ID);
    volatile uint32_t t_start = SYSTIME;
    bool pong_received = false;
    while (SYSTIME - t_start < 10000 && !pong_received)
    {
      dmxl_process_rings();
      if (g_dmxl_ports[i].comms_state != DMXL_CS_PING)
        pong_received = true;
    }
    if (pong_received)
    {
      printf("received rx @ 250 kbit from dmxl %d\r\n", i);
      continue; // hooray. it's already at the target baud rate
    }
    else
    {
      printf("no rx received from dmxl %d... trying 57600 baud...\r\n", i);
      // this is the default out-of-the-box setup
      dmxl_set_usart_baud(i, 57600);
      uint8_t baud_code = 7; // the dynamixel code for 250000 baud
      dmxl_write_data(i, DMXL_DEFAULT_ID, 1, 0x04, &baud_code);
      delay_ms(500);
      dmxl_set_usart_baud(i, 250000);
    }
  }
}

static void dmxl_tx(const uint8_t port_idx,
                    const uint8_t *payload, const uint8_t payload_len)
{
  if (port_idx >= NUM_DMXL)
    return; // bogus port index
  dmxl_port_t *dp = &g_dmxl_ports[port_idx];
  USART_TypeDef *u = dp->uart;
  u->CR1 &= ~USART_CR1_RE; // disable the receiver during transmit
  u->CR1 |=  USART_CR1_TE; // enable the transmitter
  uint8_t csum = 0;
  for (uint8_t i = 0; i < payload_len + 3; i++)
  {
    while (!(u->SR & USART_SR_TXE)) { } // wait for tx buffer to clear
    if (i <= 1)
      u->DR = 0xff; // preamble
    else if (i == payload_len + 2)
    {
      // send checksum
      u->DR = ~csum;
    }
    else
    {
      u->DR = payload[i-2]; // send payload byte
      csum += payload[i-2];
    }
  }
  while (!(u->SR & USART_SR_TC)) { } // wait for TX to finish
  u->CR1 &= ~USART_CR1_TE; // disable the transmitter
  u->CR1 |=  USART_CR1_RE; // re-enable the receiver
  // todo: actually spin here until we get a packet back in the rx ring
}

uint8_t dmxl_send_ping(const uint8_t port_idx, const uint8_t dmxl_id)
{
  uint8_t pkt[3];
  pkt[0] = dmxl_id;
  pkt[1] = 2; // packet length
  pkt[2] = 1; // instruction: ping
  dmxl_tx(port_idx, pkt, sizeof(pkt));
  return 1; // todo
}

static void dmxl_write_data(const uint8_t port_idx, const uint8_t dmxl_id,
                            const uint8_t data_len, const uint8_t start_addr,
                            const uint8_t *data)
{
  uint8_t pkt[255];
  pkt[0] = dmxl_id;
  pkt[1] = data_len + 3;
  pkt[2] = 3; // instruction: "write data"
  pkt[3] = start_addr;
  for (int i = 0; i < data_len; i++)
    pkt[4+i] = data[i];
  dmxl_tx(port_idx, pkt, data_len + 4);
}

static void dmxl_read_data(const uint8_t port_idx, const uint8_t dmxl_id,
                           const uint8_t data_len, const uint8_t start_addr)
{
  uint8_t pkt[255];
  pkt[0] = dmxl_id;
  pkt[1] = 4; // this packet's "length" is 4
  pkt[2] = 2; // instruction: "read data"
  pkt[3] = start_addr;
  pkt[4] = data_len;
  dmxl_tx(port_idx, pkt, 5);
}

void dmxl_process_ring(const uint_fast8_t dmxl_id)
{
  const uint_fast8_t i = dmxl_id; // save typing...
  while (g_dmxl_ring_rpos[i] != g_dmxl_ring_wpos[i])
  {
    const uint8_t b = g_dmxl_ring[i][g_dmxl_ring_rpos[i]];
    //printf("dmxl %d received 0x%02x\r\n", i, b);
    if (++g_dmxl_ring_rpos[i] >= DMXL_RING_LEN)
      g_dmxl_ring_rpos[i] = 0; // wrap around
    dmxl_port_t *port = &g_dmxl_ports[i]; // save typing
    switch (port->parser_state)
    {
      case DMXL_PS_PREAMBLE_0:
        if (b == 0xff)
          port->parser_state = DMXL_PS_PREAMBLE_1;
        break;
      case DMXL_PS_PREAMBLE_1:
        if (b == 0xff)
          port->parser_state = DMXL_PS_ID;
        else
          port->parser_state = DMXL_PS_PREAMBLE_0;
        break;
      case DMXL_PS_ID:
        port->rx_checksum = b;
        port->parser_state = DMXL_PS_LENGTH; // ignore ID (all the same)
        break;
      case DMXL_PS_LENGTH:
        port->rx_pkt_len = b - 2;
        port->rx_checksum += b;
        port->parser_state = DMXL_PS_ERROR;
        break;
      case DMXL_PS_ERROR:
        g_state.dynamixel_error_status[i] = b; // save for global state
        port->rx_checksum += b;
        port->rx_pkt_writepos = 0;
        if (port->rx_pkt_len)
          port->parser_state = DMXL_PS_PARAMETER;
        else
          port->parser_state = DMXL_PS_CHECKSUM;
        break;
      case DMXL_PS_PARAMETER:
        port->rx_checksum += b;
        g_dmxl_rx_pkt[i][port->rx_pkt_writepos] = b;
        if (port->rx_pkt_writepos == port->rx_pkt_len - 1)
          port->parser_state = DMXL_PS_CHECKSUM;
        port->rx_pkt_writepos++;
        break;
      case DMXL_PS_CHECKSUM:
        if (((uint8_t)(~port->rx_checksum)) == b)
        {
          /*
             printf("checksum passed. received %d bytes\r\n", port->rx_pkt_len);
             for (int j = 0; j < port->rx_pkt_len; j++)
             printf("  0x%02x\r\n", g_dmxl_rx_pkt[i][j]);
           */
          switch (port->comms_state)
          {
            case DMXL_CS_POLL_STATE:
              g_state.dynamixel_angles[i] =
                (((uint16_t)g_dmxl_rx_pkt[i][1]) << 8) |
                (((uint16_t)g_dmxl_rx_pkt[i][0])     ) ;
              //printf("dmxl %d angle = %d\r\n", (int)i, (int)g_state.dynamixel_angles[i]);
              g_state.dynamixel_speeds[i] =
                (((uint16_t)g_dmxl_rx_pkt[i][3]) << 8) |
                (((uint16_t)g_dmxl_rx_pkt[i][2])     ) ;
              g_state.dynamixel_loads[i] =
                (((uint16_t)g_dmxl_rx_pkt[i][5]) << 8) |
                (((uint16_t)g_dmxl_rx_pkt[i][4])     ) ;
              g_state.dynamixel_voltages[i]     = g_dmxl_rx_pkt[i][6];
              g_state.dynamixel_temperatures[i] = g_dmxl_rx_pkt[i][7];
              break;
            case DMXL_CS_POLL_DEBRIS:
              break;
            default:
              break;
          }
        }
        else
          printf("checksum failed: local 0x%02x != received 0x%02x\r\n",
              (uint8_t)~port->rx_checksum, b);
        port->parser_state = DMXL_PS_PREAMBLE_0;
        port->comms_state = DMXL_CS_IDLE;
        break;
      default:
        printf("woah there partner. unexpected dmxl rx state!\r\n");
        port->parser_state = DMXL_PS_PREAMBLE_0;
        break;
    }
  }
}

void dmxl_process_rings()
{
  for (uint_fast8_t i = 0; i < NUM_DMXL; i++)
    dmxl_process_ring(i);
}

// See Dynamixel addresses here: http://support.robotis.com/en/product/dynamixel/mx_series/mx-28.htm
void dmxl_set_led(const uint8_t port_idx, const uint8_t dmxl_id,
                  const uint8_t enable)
{
  uint8_t d = enable ? 1 : 0;
  dmxl_write_data(port_idx, dmxl_id, 1, 25, &d); // 25 = LED address
}


void dmxl_set_torque_enable(const uint8_t port_idx, const uint8_t dmxl_id,
                            const uint8_t enable)
{
  uint8_t d = enable ? 1 : 0;
  dmxl_write_data(port_idx, dmxl_id, 1, 24, &d);
}

void dmxl_set_angle_limits(const uint8_t port_idx, const uint8_t dmxl_id,
                           const uint16_t cw_limit, const uint16_t ccw_limit)
{
  uint8_t d[4];
  d[0] = cw_limit & 0xff;
  d[1] = (cw_limit >> 8) & 0xff;
  d[2] = ccw_limit & 0xff;
  d[3] = (ccw_limit >> 8) & 0xff;
  dmxl_write_data(port_idx, dmxl_id, 4, 6, d);
}

void dmxl_set_res_divider(const uint8_t port_idx, const uint8_t dmxl_id,
                          const uint8_t res_divider)
{
  dmxl_write_data(port_idx, dmxl_id, 1, 22, &res_divider);
}

void dmxl_set_multiturn_offset(const uint8_t port_idx, const uint8_t dmxl_id,
                               const uint16_t offset)
{
  uint8_t d[2];
  d[0] = offset & 0xff;
  d[1] = (offset >> 8) & 0xff;
  dmxl_write_data(port_idx, dmxl_id, 2, 20, d);
}

void dmxl_set_speed_dir(const uint8_t port_idx, const uint8_t dmxl_id,
                        const uint16_t speed, const uint8_t dir)
{
  uint8_t d[2];
  d[0] = speed & 0xff;
  d[1] = ((speed >> 8) & 0x03) | (dir ? 0x04 : 0);
  dmxl_write_data(port_idx, dmxl_id, 2, 32, d);
}

void dmxl_set_control_mode(const uint8_t port_idx,
                           const dmxl_control_mode_t control_mode)
{
  //printf("dmxl_set_control_mode %d %d\r\n", port_idx, (int)control_mode);
  if (port_idx >= NUM_DMXL)
    return;
  if (control_mode == DMXL_CM_IDLE || control_mode == DMXL_CM_VELOCITY)
  {
    dmxl_set_led(port_idx, DMXL_DEFAULT_ID, 0);
    delay_us(1);
    dmxl_set_torque_enable(port_idx, DMXL_DEFAULT_ID, 0);
  }
  else if (control_mode == DMXL_CM_POSITION)
  {
    dmxl_set_led(port_idx, DMXL_DEFAULT_ID, 1);
    delay_us(1);
    dmxl_set_torque_enable(port_idx, DMXL_DEFAULT_ID, 1);
    delay_us(1);
    dmxl_set_angle_limits(port_idx, DMXL_DEFAULT_ID, 4095, 4095);  // Enables multi-turn mode w/ position control
    // http://support.robotis.com/en/product/dynamixel/mx_series/mx-64.htm#Actuator_Address_0B1
    if (divider_is_sent[port_idx] == 0) {
      delay_us(1);
      dmxl_set_res_divider(port_idx, DMXL_DEFAULT_ID, 4);
      delay_us(1);
      dmxl_set_multiturn_offset(port_idx, DMXL_DEFAULT_ID, 13000);  // Places motor close enough to middle of 0-28672 range
      divider_is_sent[port_idx] = 1;
    }
  }
  dmxl_control_mode = control_mode;
}

void dmxl_set_control_target(const uint8_t port_idx,
                             const uint16_t target)
{
  //printf("dmxl_set_control_target %d %d\r\n", port_idx, (int)target);
  if (port_idx >= NUM_DMXL)
    return;
  uint8_t d[2];
  d[0] = target & 0xff;
  d[1] = (target >> 8) & 0xff;
  uint8_t start_addr = 30;  // position control
  if (dmxl_control_mode == DMXL_CM_VELOCITY) {
    start_addr = 32; // velocity control
  }
  dmxl_write_data(port_idx, DMXL_DEFAULT_ID, 2, start_addr, d);
}

void dmxl_set_all_control_targets(const uint16_t *targets)
{
  // set all control targets simultaneously
  static uint8_t pkts[NUM_DMXL][12];
  for (int i = 0; i < NUM_DMXL; i++)
  {
    USART_TypeDef *u = g_dmxl_ports[i].uart;
    u->CR1 &= ~USART_CR1_RE; // disable the receiver during transmit
    u->CR1 |=  USART_CR1_TE; // enable the transmitter
    pkts[i][0] = 0xff;
    pkts[i][1] = 0xff;
    pkts[i][2] = DMXL_DEFAULT_ID;
    pkts[i][3] = 5; // write 2 bytes + 3 byte overhead
    pkts[i][4] = 3; // instruction: "write data"
    if (dmxl_control_mode == DMXL_CM_VELOCITY) {
      pkts[i][5] = 32; // start address: control target (velocity)
    } else {
      pkts[i][5] = 30; // start address: control target (position)
    }
    pkts[i][6] = targets[i] & 0xff; // LSB of control target
    pkts[i][7] = (targets[i] >> 8) & 0xff; // MSB of control target
    pkts[i][8] = 0; // checksum
    for (int j = 2; j < 8; j++)
      pkts[i][8] += pkts[i][j];
    pkts[i][8] = ~pkts[i][8]; // invert checksum
  }
  for (int j = 0; j < 9; j++)
  {
    for (int i = 0; i < NUM_DMXL; i++)
    {
      USART_TypeDef *u = g_dmxl_ports[i].uart;
      while (!(u->SR & USART_SR_TXE)) { } // wait for tx buffer to clear
      u->DR = pkts[i][j];
    }
  }
  for (int i = 0; i < NUM_DMXL; i++)
  {
    USART_TypeDef *u = g_dmxl_ports[i].uart;
    while (!(u->SR & USART_SR_TC)) { } // wait for last TX to finish
    u->CR1 &= ~USART_CR1_TE; // disable the transmitter
    u->CR1 |=  USART_CR1_RE; // re-enable the receiver
  }
}

void dmxl_poll()
{
  // spin through and poll all their angles, velocities, currents,
  // voltages, temperatures, etc.
  for (uint_fast8_t i = 0; i < NUM_DMXL; i++)
  {
    g_dmxl_ports[i].comms_state = DMXL_CS_POLL_STATE;
    dmxl_read_data(i, DMXL_DEFAULT_ID, 8, 36);
  }
  volatile uint32_t t_start = SYSTIME;
  while (SYSTIME - t_start < 10000)
  {
    dmxl_process_rings();
    // also spin around the ethernet rings here? probably not.
    int all_done = 1;
    for (uint_fast8_t i = 0; i < NUM_DMXL; i++)
      if (g_dmxl_ports[i].comms_state == DMXL_CS_POLL_STATE)
        all_done = 0;
    if (all_done)
      break;
  }
  // set the comms state to "idle" even if we didn't hear back from it by now
  for (uint_fast8_t i = 0; i < NUM_DMXL; i++)
    g_dmxl_ports[i].comms_state = DMXL_CS_IDLE;
}

void dmxl_poll_nonblocking_tick(const uint8_t dmxl_port)
{
  if (dmxl_port >= NUM_DMXL)
    return; // let's not corrupt memory.
  dmxl_async_poll_state_t *ps = &dmxl_poll_states[dmxl_port]; // save typing
  dmxl_port_t *dp = &g_dmxl_ports[dmxl_port];
  USART_TypeDef *u = dp->uart;
  static uint8_t dmxl_txbuf[NUM_DMXL][256];
  static uint8_t dmxl_txbuf_readpos[NUM_DMXL] = {0};
  static uint32_t dmxl_rx_start_time[NUM_DMXL] = {0};
  switch (*ps)
  {
    case DPS_DONE: // poll start
      dmxl_rx_start_time[dmxl_port] = SYSTIME;
      *ps = DPS_WAIT; // give any outbound command some time to be processed
      break;
    case DPS_WAIT:
      if (SYSTIME - dmxl_rx_start_time[dmxl_port] > 5000)
      {
        u->CR1 &= ~USART_CR1_RE; // disable the receiver during transmit
        u->CR1 |=  USART_CR1_TE; // enable the transmitter
        dmxl_txbuf[dmxl_port][0] = 0xff; // header
        dmxl_txbuf[dmxl_port][1] = 0xff; // more header
        dmxl_txbuf[dmxl_port][2] = DMXL_DEFAULT_ID;
        dmxl_txbuf[dmxl_port][3] = 4;  // packet length = 4
        dmxl_txbuf[dmxl_port][4] = 2;  // instruction: "read data"
        dmxl_txbuf[dmxl_port][5] = 36; // start address
        dmxl_txbuf[dmxl_port][6] = 8;  // number of bytes to read
        uint8_t csum = 0;
        for (int i = 2; i < 7; i++)
          csum += dmxl_txbuf[dmxl_port][i];
        dmxl_txbuf[dmxl_port][7] = ~csum;
        u->DR; // read any garbage on the RX register
        u->DR = dmxl_txbuf[dmxl_port][0]; // kick it off
        dmxl_txbuf_readpos[dmxl_port] = 0;
        *ps = DPS_POLL_TX;
      }
      break;
    case DPS_POLL_TX:
      if (u->SR & USART_SR_TXE) // is there room in the TX buffer ?
      {
        if (dmxl_txbuf_readpos[dmxl_port] < 7)
        {
          dmxl_txbuf_readpos[dmxl_port]++;
          u->DR = dmxl_txbuf[dmxl_port][dmxl_txbuf_readpos[dmxl_port]];
        }
        else if (u->SR & USART_SR_TC) // last byte must fully complete
        {
          u->CR1 &= ~USART_CR1_TE; // disable the transmitter
          u->CR1 |=  USART_CR1_RE; // re-enable the transmitter
          dmxl_rx_start_time[dmxl_port] = SYSTIME;
          g_dmxl_ports[dmxl_port].comms_state = DMXL_CS_POLL_STATE;
          *ps = DPS_POLL_RX;
        }
      }
      break;
    case DPS_POLL_RX:
      dmxl_process_ring(dmxl_port);
      if (g_dmxl_ports[dmxl_port].comms_state != DMXL_CS_POLL_STATE)
        *ps = DPS_DONE; // hooray, we received a full state message
      if (SYSTIME - dmxl_rx_start_time[dmxl_port] > 10000) // wait at most 10ms
        *ps = DPS_DONE; // time to give up. gotta know when to fold em
      break;
    default:
      *ps = DPS_DONE; // shouldn't get here
      break;
  }
}

bool dmxl_all_available()
{
  for (int i = 0; i < NUM_DMXL; i++)
    if (dmxl_poll_states[i] != DPS_DONE)
      return false;
  return true;
}
