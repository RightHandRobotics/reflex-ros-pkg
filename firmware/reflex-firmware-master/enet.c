#include "enet.h"

static volatile eth_dma_desc_t g_eth_dma_rx_desc[ETH_DMA_NRXD] ALIGN4;
static volatile eth_dma_desc_t g_eth_dma_tx_desc[ETH_DMA_NTXD] ALIGN4;
static volatile uint8_t g_eth_dma_rx_buf[ETH_DMA_NRXD][ETH_NBUF] ALIGN4;
static volatile uint8_t g_eth_dma_tx_buf[ETH_DMA_NTXD][ETH_NBUF] ALIGN4;
static volatile eth_dma_desc_t *g_eth_dma_rx_next_desc = &g_eth_dma_rx_desc[0];
static volatile eth_dma_desc_t *g_eth_dma_tx_next_desc = &g_eth_dma_tx_desc[0];

static volatile uint8_t  g_eth_rxpool[ETH_RAM_RXPOOL_LEN] ALIGN4;
static volatile uint16_t g_eth_rxpool_wpos = 0;
static volatile uint8_t *g_eth_rxpool_start[ETH_RAM_RXPOOL_NPTR] ALIGN4;
static volatile uint16_t g_eth_rxpool_len[ETH_RAM_RXPOOL_NPTR] ALIGN4;
static volatile uint16_t g_eth_rxpool_ptrs_wpos = 0;
static volatile uint16_t g_eth_rxpool_ptrs_rpos = 0;

uint16_t enet_read_phy_reg(const uint8_t reg_idx)
{
  while (ETH->MACMIIAR & ETH_MACMIIAR_MB) { } // ensure MII is idle
  ETH->MACMIIAR = (ENET_PHY_ADDR << 11) |
                  ((reg_idx & 0x1f) << 6) |
                  ETH_MACMIIAR_CR_Div102  | // clock divider
                  ETH_MACMIIAR_MB;
  while (ETH->MACMIIAR & ETH_MACMIIAR_MB) { } // spin waiting for MII to finish
  return ETH->MACMIIDR & 0xffff;
}

void enet_write_phy_reg(const uint8_t reg_idx, const uint16_t reg_val)
{
  while (ETH->MACMIIAR & ETH_MACMIIAR_MB) { } // ensure MII is idle
  ETH->MACMIIDR = reg_val; // set the outgoing data word
  ETH->MACMIIAR = (ENET_PHY_ADDR << 11)   |
                  ((reg_idx & 0x1f) << 6) |
                  ETH_MACMIIAR_CR_Div102  | // MDC clock divider
                  ETH_MACMIIAR_MW         | // set the write bit
                  ETH_MACMIIAR_MB; // start it up
  while (ETH->MACMIIAR & ETH_MACMIIAR_MB) { } // spin waiting for MII to finish
  uint16_t readback_val = enet_read_phy_reg(reg_idx);
  if (readback_val != reg_val)
  {
    printf("woah there. tried to write 0x%04x to reg %02d but it read back %04x\r\n",
           reg_val, reg_idx, readback_val);
  }
}

void enet_init()
{
  printf("enet_init()\r\n");
  // set up the pins on port a
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN     |  // and all the i/o ports we
                  RCC_AHB1ENR_GPIOBEN     |  // will be using to talk to
                  RCC_AHB1ENR_GPIOCEN;       // the ethernet PHY

  GPIOA->MODER |= (2 << (PORTA_ETH_REFCLK * 2)) |
                  (2 << (PORTA_ETH_MDIO   * 2)) |
                  (2 << (PORTA_ETH_CRSDV  * 2)); // set these guys as AF pins
  GPIOA->AFR[0] |= (11 << (PORTA_ETH_REFCLK * 4)) |
                   (11 << (PORTA_ETH_MDIO   * 4)) |
                   (11 << (PORTA_ETH_CRSDV  * 4));

  // set up the ethernet pins on port b
  GPIOB->MODER |= (2 << (PORTB_ETH_TXEN * 2)) |
                  (2 << (PORTB_ETH_TXD0 * 2)) |
                  (2 << (PORTB_ETH_TXD1 * 2));
  GPIOB->AFR[1] |= (11 << ((PORTB_ETH_TXEN - 8) * 4)) |
                   (11 << ((PORTB_ETH_TXD0 - 8) * 4)) |
                   (11 << ((PORTB_ETH_TXD1 - 8) * 4));
  GPIOB->OSPEEDR |= (3 << (PORTB_ETH_TXEN * 2)) |
                    (3 << (PORTB_ETH_TXD0 * 2)) |
                    (3 << (PORTB_ETH_TXD1 * 2)); // make the tx pins go fast
  GPIOB->MODER |= (1 << (PORTB_PHY_RESET * 2)); // reset = gpio output pin

  // set up the ethernet pins on port c
  GPIOC->MODER  |= ( 2 << (PORTC_ETH_MDC  * 2)) |
                   ( 2 << (PORTC_ETH_RXD0 * 2)) |
                   ( 2 << (PORTC_ETH_RXD1 * 2));
  GPIOC->AFR[0] |= (11 << (PORTC_ETH_MDC  * 4)) |
                   (11 << (PORTC_ETH_RXD0 * 4)) |
                   (11 << (PORTC_ETH_RXD1 * 4));

  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // enable the sysconfig block
  RCC->AHB1RSTR |= RCC_AHB1RSTR_ETHMACRST;
  for (volatile int i = 0; i < 1000; i++) { } // wait for sysconfig to come up
  // hold the MAC in reset while we set it to RMII mode
  for (volatile int i = 0; i < 1000; i++) { } // wait for sysconfig to come up
  SYSCFG->PMC |= SYSCFG_PMC_MII_RMII_SEL; // set the MAC in RMII mode
  for (volatile int i = 0; i < 100000; i++) { } // wait for sysconfig to come up
  RCC->AHB1ENR |= RCC_AHB1ENR_ETHMACRXEN  |
                  RCC_AHB1ENR_ETHMACTXEN  |
                  RCC_AHB1ENR_ETHMACEN    ;  // turn on ur ethernet plz
  for (volatile int i = 0; i < 100000; i++) { } // wait
  RCC->AHB1RSTR &= ~RCC_AHB1RSTR_ETHMACRST; // release MAC reset
  for (volatile int i = 0; i < 100000; i++) { } // wait
  RCC->AHB1RSTR |= RCC_AHB1RSTR_ETHMACRST;
  for (volatile int i = 0; i < 100000; i++) { } // wait
  RCC->AHB1RSTR &= ~RCC_AHB1RSTR_ETHMACRST; // release MAC reset
  for (volatile int i = 0; i < 100000; i++) { } // wait for sysconfig ... (?)

  ETH->DMABMR |= ETH_DMABMR_SR;
  for (volatile uint32_t i = 0; i < 100000; i++) { }
  while (ETH->DMABMR & ETH_DMABMR_SR) { } // wait for it to reset
  for (volatile uint32_t i = 0; i < 100000; i++) { }
  ETH->DMAOMR |= ETH_DMAOMR_FTF; // flush DMA
  while (ETH->DMAOMR & ETH_DMAOMR_FTF) { } // wait for it to flush

  // now, configure the ethernet peripheral
  ETH->MACCR |= 0x02000000 | // CSTF = strip FCS. why isn't there a symbol ?
                ETH_MACCR_FES  | // enable 100 mbit mode
                ETH_MACCR_DM   | // full duplex
                ETH_MACCR_IPCO | // ipv4 checksum auto-generation for RX
                ETH_MACCR_APCS;  // automatically remove pad+CRC from frames
  ETH->MACFFR |= ETH_MACFFR_RA; // for now, don't try to filter in hardware
  // generate a decent reset pulse now
  GPIOB->BSRRL = 1 << PORTB_PHY_RESET;
  for (volatile uint32_t i = 0; i < 100000; i++) { }
  GPIOB->BSRRH = 1 << PORTB_PHY_RESET; // assert reset (pull it low)
  for (volatile uint32_t i = 0; i < 100000; i++) { } // let some time pass
  GPIOB->BSRRL = 1 << PORTB_PHY_RESET; // de-assert reset (pull it high)
  // todo: only need to wait until registers read back something other
  // than 0xffff . then we don't have to wait as long.
  for (volatile uint32_t i = 0; i < 1000000; i++) { } // let it initialize
  printf("waiting for PHY to wake up...\r\n");
  while (enet_read_phy_reg(0) == 0xffff) { }
  for (volatile uint32_t i = 0; i < 1000000; i++) { } // let it initialize
  printf("done with PHY reset.\r\n");
  printf("setting software strap registers...\r\n");
  enet_write_phy_reg(0x09, 0x7821); // enable auto MDIX,
                                    // set INT/PWDN to be interrupt output
                                    // enable auto-negotiation
  enet_write_phy_reg(0x09, 0xf821); // exit software-strap mode
  enet_write_phy_reg(0x04, 0x0101); // only advertise 100-FD mode

  // cycle through and read a bunch of PHY registers to make sure it's alive
  for (int i = 0; i < 32; i++)
    printf("PHY reg %02d = 0x%04x\r\n", i, enet_read_phy_reg(i));

  ////////////////////////////////////////////////////////////////////////
  // set up ethernet TX descriptors
  for (int i = 0; i < ETH_DMA_NTXD; i++)
  {
    g_eth_dma_tx_desc[i].des0 = 0x00100000 | // set address-chained bit
                                0x00c00000 ; // set insert-checksum bits
    g_eth_dma_tx_desc[i].des1 = 0;
    g_eth_dma_tx_desc[i].des2 = (uint32_t)&g_eth_dma_tx_buf[i][0]; // pointer to buf
    if (i < ETH_DMA_NTXD-1)
      g_eth_dma_tx_desc[i].des3 = (uint32_t)&g_eth_dma_tx_desc[i+1]; // chain to next
    else
      g_eth_dma_tx_desc[i].des3 = (uint32_t)&g_eth_dma_tx_desc[0]; // loop to first
  }
  ////////////////////////////////////////////////////////////////////////
  // set up ethernet RX descriptors
  for (int i = 0; i < ETH_DMA_NRXD; i++)
  {
    g_eth_dma_rx_desc[i].des0 = 0x80000000; // set "own" bit = DMA has control
    g_eth_dma_rx_desc[i].des1 = 0x00004000 | // set the RCH bit = chained addr2
                            ETH_NBUF; // buffer size in addr1
    g_eth_dma_rx_desc[i].des2 = (uint32_t)&g_eth_dma_rx_buf[i][0];
    if (i < ETH_DMA_NRXD-1)
      g_eth_dma_rx_desc[i].des3 = (uint32_t)&g_eth_dma_rx_desc[i+1];
    else
      g_eth_dma_rx_desc[i].des3 = (uint32_t)&g_eth_dma_rx_desc[0];
  }

  ///////////////////////////////////////////////////////////////////////
  // set up the RAM pool for reception
  for (int i = 0; i < ETH_RAM_RXPOOL_NPTR; i++)
  {
    g_eth_rxpool_start[i] = &g_eth_rxpool[0];
    g_eth_rxpool_len[i] = 0;
    g_eth_rxpool_ptrs_wpos = 0;
    g_eth_rxpool_ptrs_rpos = 0;
  }

  ///////////////////////////////////////////////////////////////////////
  // finally, turn on the DMA machinery
  ETH->DMATDLAR = (uint32_t)&g_eth_dma_tx_desc[0]; // point TX DMA to first desc
  ETH->DMARDLAR = (uint32_t)&g_eth_dma_rx_desc[0]; // point RX DMA to first desc
  ETH->DMAOMR = ETH_DMAOMR_TSF; // enable store-and-forward mode
  /*
  ETH->DMABMR = ETH_DMABMR_AAB | ETH_DMABMR_USP |
                ETH_DMABMR_RDP_1Beat | ETH_DMABMR_RTPR_1_1 |
                ETH_DMABMR_PBL_1Beat | ETH_DMABMR_EDE;
  */
  ETH->DMAIER = ETH_DMAIER_NISE | ETH_DMAIER_RIE;
  ETH->MACCR |= ETH_MACCR_TE | // enable transmitter
                ETH_MACCR_RE;  // enable receiver
  NVIC_SetPriority(ETH_IRQn, 3);
  NVIC_EnableIRQ(ETH_IRQn);
  ETH->DMAOMR |= ETH_DMAOMR_ST | ETH_DMAOMR_SR; // enable ethernet DMA tx/rx
}

void eth_vector()
{
  volatile uint32_t dmasr = ETH->DMASR;
  ETH->DMASR = dmasr; // clear pending bits in the status register
  //printf("eth_vector()\r\n");
  if (dmasr & ETH_DMASR_RS)
  {
    // we received one or more frames. spin through and find them...
    while (!(g_eth_dma_rx_next_desc->des0 & 0x80000000))
    {
      // todo: check all of the error status bits in des0...
      const uint16_t rxn = (g_eth_dma_rx_next_desc->des0 & 0x3fff0000) >> 16;
      // see if this packet will run off the end of the buffer. if so, wrap.
      if (g_eth_rxpool_wpos + rxn >= ETH_RAM_RXPOOL_LEN)
        g_eth_rxpool_wpos = 0;
      const uint16_t wp = g_eth_rxpool_ptrs_wpos;
      g_eth_rxpool_start[wp] = &g_eth_rxpool[g_eth_rxpool_wpos];
      g_eth_rxpool_len[wp] = rxn;
      memcpy((uint8_t *)&g_eth_rxpool[g_eth_rxpool_wpos],
             (const uint8_t *)g_eth_dma_rx_next_desc->des2,
             rxn);
      //printf("ethernet rx %d into rxpool ptr %d\r\n", rxn, wp);
      g_eth_rxpool_ptrs_wpos++;
      if (g_eth_rxpool_ptrs_wpos >= ETH_RAM_RXPOOL_NPTR)
        g_eth_rxpool_ptrs_wpos = 0;
      g_eth_rxpool_wpos += rxn;

/*
      uint8_t *p = (uint8_t *)g_eth_rx_next_desc->des2;
      for (int i = 0; i < rxn; i++)
        printf("%02d: 0x%02x\r\n", i, p[i]);
*/

      g_eth_dma_rx_next_desc->des0 |= 0x80000000; // give it back to the DMA
      // advance the rx pointer for next time
      g_eth_dma_rx_next_desc = (eth_dma_desc_t *)g_eth_dma_rx_next_desc->des3;
    }
  }
  dmasr = ETH->DMASR;
  //printf("dmasr = 0x%08x\r\n", (int)dmasr);
}

enet_link_status_t enet_get_link_status()
{
  uint16_t status = enet_read_phy_reg(0x01);
  //printf("PHY status = 0x%02x\r\n", status);
  if (status & (1 << 4)) {
    err_set(ERR_ETH_REMOTE_FAULT);
  } else {
    err_unset(ERR_ETH_REMOTE_FAULT);
  }
  if (status & (1 << 5)) {
    err_unset(ERR_ETH_NEGOTIATING);
  } else {
    err_set(ERR_ETH_NEGOTIATING);
  }
  if (status & (1 << 2))
    return ENET_LINK_UP;
  return ENET_LINK_DOWN;
}

void eth_send_raw_packet(uint8_t *pkt, uint16_t pkt_len)
{
  //printf("eth tx %d\r\n", pkt_len);
  if (g_eth_dma_tx_next_desc->des0 & 0x80000000) // check the OWN bit
  {
    err_set(ERR_ETH_BUFFER_FULL);
    return; // if it's set, then we have run out of ringbuffer room. can't tx.
  }
  err_unset(ERR_ETH_BUFFER_FULL);
  /*
  printf("sending using TX descriptor %08x status 0x%08x\r\n",
         (unsigned)g_eth_tx_next_desc,
         (unsigned)g_eth_tx_next_desc->control);
 */
  uint8_t *buf = (uint8_t *)g_eth_dma_tx_next_desc->des2;
  if (pkt_len > ETH_NBUF)
    pkt_len = ETH_NBUF; // let's not blow through our packet buffer
  memcpy(buf, pkt, pkt_len);
  g_eth_dma_tx_next_desc->des1 = pkt_len;
  g_eth_dma_tx_next_desc->des0 |= 0x30000000; // LS+FS = single-buffer packet
  g_eth_dma_tx_next_desc->des0 |= 0x80000000; // give ownership to ethernet DMA
  // see if DMA is stuck because it wasn't transmitting (which will almost
  // always be the case). if it's stuck, kick it into motion again
  if ((ETH->DMASR & ETH_DMASR_TPS) == ETH_DMASR_TPS_Suspended)
  {
    ETH->DMASR = ETH_DMASR_TBUS; // clear the buffer-unavailable flag
    ETH->DMATPDR = 0; // transmit poll demand = kick it moving again
  }
  g_eth_dma_tx_next_desc = (eth_dma_desc_t *)g_eth_dma_tx_next_desc->des3;
  //uint16_t r = enet_read_phy_reg(0x17);
  //printf(" rmii status = 0x%04x\r\n", (unsigned)r);
}

// todo: find ways for this to be overridden on CPU's with built-ins for this
uint16_t eth_htons(const uint16_t x)
{
  return ((x & 0xff) << 8) | ((x >> 8) & 0xff);
}

uint32_t eth_htonl(const uint32_t x)
{
  return ((x & 0x000000ff) << 24)  |
         ((x & 0x0000ff00) << 8)   |
         ((x & 0x00ff0000) >> 8)   |
         ((x & 0xff000000) >> 24);
}

#define ETH_MAC_LEN 6
typedef struct
{
  uint8_t  dest_addr[ETH_MAC_LEN];
  uint8_t  source_addr[ETH_MAC_LEN];
  uint16_t ethertype : 16;
} __attribute__((packed)) eth_eth_header_t;

#define ETH_ETHERTYPE_IP    0x0800
#define ETH_ETHERTYPE_ARP   0x0806

typedef struct
{
  eth_eth_header_t eth;
  uint8_t  header_len   :  4;
  uint8_t  version      :  4;
  uint8_t  ecn          :  2;
  uint8_t  diff_serv    :  6;
  uint16_t len          : 16;
  uint16_t id           : 16;
  uint16_t flag_frag    : 16;
  uint8_t  ttl          :  8;
  uint8_t  proto        :  8;
  uint16_t checksum     : 16;
  uint32_t source_addr  : 32;
  uint32_t dest_addr    : 32;
} __attribute__((packed)) eth_ip_header_t;

#define ETH_IP_HEADER_LEN     5
#define ETH_IP_VERSION        4
#define ETH_IP_DONT_FRAGMENT  0x4000

#define ETH_IP_PROTO_ICMP  0x01
#define ETH_IP_PROTO_UDP   0x11

typedef struct
{
  eth_ip_header_t ip;
  uint16_t source_port;
  uint16_t dest_port;
  uint16_t len;
  uint16_t checksum;
} __attribute__((packed)) eth_udp_header_t;

static uint8_t  g_eth_udpbuf[1500] __attribute__((aligned(8))) = {0};
static uint8_t  g_eth_src_mac[6] = { 0xa4, 0xf3, 0xc1, 0x00, 0x01, 0x00 };
static uint32_t g_eth_src_ip = 0x0a636363; // todo... not this.

void enet_send_udp_mcast(const uint32_t mcast_ip, const uint16_t mcast_port,
                         const uint8_t *payload, const uint16_t payload_len)
{
  uint8_t dest_mac[6] = { 0x01, 0x00, 0x5e,
                          (uint8_t)((mcast_ip & 0xff0000) >> 16),
                          (uint8_t)((mcast_ip & 0x00ff00) >>  8),
                          (uint8_t) (mcast_ip & 0x0000ff) };
  enet_send_udp_ucast(dest_mac, mcast_ip, mcast_port,
                      g_eth_src_ip, mcast_port,
                      payload, payload_len);
}

void enet_send_udp_ucast(const uint8_t *dest_mac,
                         const uint32_t dest_ip, const uint16_t dest_port,
                         const uint32_t source_ip, const uint16_t source_port,
                         const uint8_t *payload, const uint16_t payload_len)
{
  eth_udp_header_t *h = (eth_udp_header_t *)&g_eth_udpbuf[0];
  for (int i = 0; i < 6; i++)
  {
    h->ip.eth.dest_addr[i] = dest_mac[i];
    h->ip.eth.source_addr[i] = g_eth_src_mac[i];
  }
  h->ip.eth.ethertype = eth_htons(ETH_ETHERTYPE_IP);
  h->ip.header_len = ETH_IP_HEADER_LEN;
  h->ip.version = ETH_IP_VERSION; // ipv4
  h->ip.ecn = 0;
  h->ip.diff_serv = 0;
  h->ip.len = eth_htons(20 + 8 + payload_len);
  h->ip.id = 0;
  h->ip.flag_frag = eth_htons(ETH_IP_DONT_FRAGMENT);
  h->ip.ttl = 1; // not sure here...
  h->ip.proto = ETH_IP_PROTO_UDP;
  h->ip.checksum = 0; // will be filled by the ethernet TX machinery
  h->ip.dest_addr = eth_htonl(dest_ip);
  h->ip.source_addr = eth_htonl(source_ip); //); // todo: something else
  h->dest_port = eth_htons(dest_port);
  h->source_port = eth_htons(source_port); //1234;
  h->len = eth_htons(8 + payload_len);
  h->checksum = 0; // will be filled by the ethernet TX machinery
  memcpy(g_eth_udpbuf + sizeof(eth_udp_header_t), payload, payload_len);
  eth_send_raw_packet(g_eth_udpbuf, sizeof(eth_udp_header_t) + payload_len);
  /*
  uint8_t raw_test_pkt[128] =
  { 0x01, 0x00, 0x5e, 0x00, 0x00, 0x7b, 0x90, 0x2b,
    0x34, 0x39, 0xb3, 0x2e, 0x08, 0x00, 0x45, 0x00,
    0x00, 0x2b, 0x00, 0x00, 0x40, 0x00, 0x01, 0x11,
    0xd7, 0x1e, 0xc0, 0xa8, 0x01, 0x80, 0xe0, 0x00,
    0x00, 0x7b, 0xcc, 0xad, 0x04, 0xd4, 0x00, 0x17,
    0x00, 0x00, 0x01, 0x01, 0x00, 0x0b, 0x00, 0x00,
    0x08, 0xca, 0xfe, 0xbe, 0xef, 0x12, 0x34, 0x56,
    0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x7b, 0x94, 0x60, 0x0f };
  eth_send_raw_packet(raw_test_pkt, 68); //sizeof(reg_idx) + payload_len);
  */
}

uint_fast8_t enetRX()
{
  uint_fast8_t num_pkts_rx = 0;
  while (g_eth_rxpool_ptrs_wpos != g_eth_rxpool_ptrs_rpos)
  {
    const uint16_t rp = g_eth_rxpool_ptrs_rpos;
    const uint8_t *start = (const uint8_t *)g_eth_rxpool_start[rp];
    const uint16_t len = g_eth_rxpool_len[rp];
    const eth_eth_header_t *e = (const eth_eth_header_t *)start;
    uint8_t unicast_match = 1;
    uint8_t multicast_match = 1;
    uint8_t broadcast_match = 1;
    
    //printf("eth rxpool wpos = %d rpos = %d start %d len %d\r\n",
    //       g_eth_rxpool_ptrs_wpos,
    //       rp, start - g_eth_rxpool, len);
    // see if it's addressed to us
    
    /*
    printf("rx mac: %02x:%02x:%02x:%02x:%02x:%02x\r\n",
           e->dest_addr[0], e->dest_addr[1], e->dest_addr[2],
           e->dest_addr[3], e->dest_addr[4], e->dest_addr[5]);
    */
    for (int i = 0; i < 6; i++)
    {
      if (e->dest_addr[i] != g_eth_src_mac[i])
        unicast_match = 0;
      if (e->dest_addr[i] != 0xff)
        broadcast_match = 0;
    }
    if (e->dest_addr[0] != 0x01 || e->dest_addr[1] != 0x00 || e->dest_addr[2] != 0x5e)
      multicast_match = 0;
    //printf("  ucast_match = %d, bcast_match = %d, mcast_match = %d\r\n",
    //       unicast_match, broadcast_match, multicast_match);
    //printf("dispatch @ %8u\r\n", (unsigned)SYSTIME);
    if (unicast_match || multicast_match || broadcast_match)
      num_pkts_rx += eth_dispatch_eth(start, len) ? 1 : 0;
    if (++g_eth_rxpool_ptrs_rpos >= ETH_RAM_RXPOOL_NPTR)
      g_eth_rxpool_ptrs_rpos = 0;
  }
  return num_pkts_rx;
}

bool eth_dispatch_eth(const uint8_t *data, const uint16_t len)
{
  // dispatch according to protocol
  const eth_eth_header_t *e = (const eth_eth_header_t *)data;
  switch (eth_htons(e->ethertype))
  {
    case ETH_ETHERTYPE_IP:
      return eth_dispatch_ip(data, len);
    default:
      return false;
  }
}

bool eth_dispatch_ip(const uint8_t *data, const uint16_t len)
{
  const eth_ip_header_t *ip = (const eth_ip_header_t *)data;
  if (ip->version != 4) // we only handle ipv4 (for now...)
    return false;
  // if it's unicast, verify our IP address, otherwise ignore the packet
  if (ip->eth.dest_addr[0] == g_eth_src_mac[0])
    if (ip->source_addr != eth_htonl(g_eth_src_ip))
      return false;
  if (ip->proto == ETH_IP_PROTO_UDP)
    return eth_dispatch_udp(data, len);
  return false; // if we get here, we aren't smart enough to handle this packet
}

bool eth_dispatch_udp(const uint8_t *data, const uint16_t len)
{
  const eth_udp_header_t *udp = (const eth_udp_header_t *)data;
  const uint16_t port = eth_htons(udp->dest_port);
  const uint16_t payload_len = eth_htons(udp->len) - 8;
  const uint8_t *payload = data + sizeof(eth_udp_header_t);
  //printf("  udp len: %d\r\n", udp_payload_len);
  if (payload_len > len - sizeof(eth_udp_header_t))
    return false; // ignore fragmented UDP packets.
  //printf("dispatch udp @ %8u\r\n", (unsigned)SYSTIME);

  //printf("dispatch udp: port = %d  payload_len = %d\r\n", port, payload_len);
  /*
  for (int i = 0; i < payload_len; i++)
  {
    printf("  %02d: %02x\r\n", i, payload[i]);
  }
  */
  if (port == 11333 && payload_len > 0)
  {
    const uint8_t cmd = payload[0];
    //printf("  enet rx cmd = 0x%02x\r\n", cmd);
    if (cmd == 0) // CORRECT
    {
      printf("Received printStatusCommand...\n");

    }
    else if (cmd == 1 && payload_len >= 5)
    {
      /*
      printf("    modes: %d %d %d %d\r\n",
             payload[1], payload[2], payload[3], payload[4]);
      */
      for (int i = 0; i < NUM_DMXL; i++)
        dmxl_set_control_mode(i, (dmxl_control_mode_t)payload[1+i]);
      delay_ms(1); // be sure they control mode messages get through
      return true;
    }
    else if (cmd == 2 && payload_len >= 9)
    {
      uint16_t targets[NUM_DMXL];
      for (int i = 0; i < NUM_DMXL; i++)
        targets[i] = (payload[1+2*i] << 8) | payload[2+2*i];
      /*
      printf("targets: %06d %06d %06d %06d\r\n",
             targets[0], targets[1], targets[2], targets[3]);
      */
      /*
      for (int i = 0; i < NUM_DMXL; i++)
        dmxl_set_control_target(i, targets[i]);
      */
      dmxl_set_all_control_targets(targets);
      //dmxl_set_control_target(0, targets[0]); // debugging... just do #0
      return true;
    }
  }
  // if we get here, we haven't handled this packet
  return false;
}

// todo: be smarter about multicast group choice
#define MCAST_IP 0xe000007c
#define MCAST_IP 0xe000007c

void enetTX()
{
  volatile state_t tx_state = handState; // make a local copy to ensure coherence
  enet_send_udp_mcast(MCAST_IP, 11333, (uint8_t *)&tx_state, sizeof(tx_state));

  // if ... // CORRECT, this is to send debug information via serial
  // volatile state_t tx_status = handStatus; // make a local copy to ensure coherence
  // enet_send_udp_mcast(MCAST_IP, 11333, (uint8_t *)&tx_state, sizeof(tx_state));
}

void ethernetService(void)
{
  if (enet_get_link_status() == ENET_LINK_UP)
  {
    err_unset(ERR_NO_ETHERNET);
    enetTX();
  }
  else
  {
    err_set(ERR_NO_ETHERNET);
  }
  enetRX();
}