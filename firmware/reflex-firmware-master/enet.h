#ifndef ENET_H
#define ENET_H

#include <stdint.h>
#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
#include "error.h"
#include "state.h"
#include "systime.h"
#include "delay.h"
#include <stdbool.h>
#include <dmxl.h>

// declare the pin numbers
#define PORTA_ETH_REFCLK 1
#define PORTA_ETH_MDIO   2
#define PORTA_ETH_CRSDV  7
#define PORTB_ETH_TXEN   11
#define PORTB_ETH_TXD0   12
#define PORTB_ETH_TXD1   13
#define PORTB_PHY_RESET  14
#define PORTC_ETH_MDC    1
#define PORTC_ETH_RXD0   4
#define PORTC_ETH_RXD1   5

// address is hard-wired on board using internal chip pullups.
#define ENET_PHY_ADDR 0x01

#define ETH_NBUF 2048
#define ETH_DMA_NRXD 16
#define ETH_DMA_NTXD  4

typedef struct
{
  uint32_t des0;
  uint32_t des1;
  uint32_t des2;
  uint32_t des3;
} eth_dma_desc_t;

#define ALIGN4 __attribute__((aligned(4)));

#define ETH_RAM_RXPOOL_LEN  16384
#define ETH_RAM_RXPOOL_NPTR   128

///////////////////////////////////////////////////////////////////////////
// local functions
void eth_send_raw_packet(uint8_t *pkt, uint16_t pkt_len); // static
bool eth_dispatch_eth(const uint8_t *data, const uint16_t len); // static
bool eth_dispatch_ip(const uint8_t *data, const uint16_t len); // static
bool eth_dispatch_udp(const uint8_t *data, const uint16_t len); // static

///////////////////////////////////////////////////////////////////////////

void enet_init();

typedef enum { ENET_LINK_DOWN, ENET_LINK_UP } enet_link_status_t;
enet_link_status_t enet_get_link_status();

void enet_send_udp_ucast(const uint8_t *dest_mac,
                           const uint32_t dest_ip, const uint16_t dest_port,
                           const uint32_t source_ip, const uint16_t source_port,
                           const uint8_t *payload, const uint16_t payload_len);

void enet_send_udp_mcast(const uint32_t mcast_ip, const uint16_t mcast_port,
                         const uint8_t *payload, const uint16_t payload_len);

void enet_write_phy_reg(const uint8_t reg_idx, const uint16_t reg_val);

uint_fast8_t enetRX(void);
void enetTX(void);
void ethernetService(void);
#endif

