#include "enc.h"
#include "stm32f4xx.h"
#include <stdio.h>
#include "pin.h"
#include "state.h"
#include "systime.h"
#include "error.h"

/////////////////////////////////////////////////////////////////////
// encoder daisy-chain setup
//
// spi4 cs   = pe11, af 5
// spi4 sclk = pe12, af 5
// spi4 miso = pe13, af 5
// spi4 mosi = pe14, af 5

#define PORTE_ENC_CS   11
#define PORTE_ENC_SCLK 12
#define PORTE_ENC_MISO 13
#define PORTE_ENC_MOSI 14

#define TRUE 1
#define FALSE 0
// The number of times we get the same reading before we light an
// LED
static const uint8_t LIGHT_LED_THRESHOLD = 100;

enc_async_poll_state_t enc_poll_state = { EPS_DONE };

void enc_init()
{
  printf("enc_init()\r\n");
  RCC->APB2ENR |= RCC_APB2ENR_SPI4EN; // turn on SPI4

  pin_set_output(GPIOE, PORTE_ENC_CS);
  pin_set_output_level(GPIOE, PORTE_ENC_CS, 1);
  pin_set_alternate_function(GPIOE, PORTE_ENC_SCLK, 5);
  pin_set_alternate_function(GPIOE, PORTE_ENC_MISO, 5);
  pin_set_alternate_function(GPIOE, PORTE_ENC_MOSI, 5);

  // spi4 is running from a 84 MHz pclk. set it up with
  // sclk = pclk/128 to be super slow for now.
  SPI4->CR1 = SPI_CR1_DFF  | // 16-bit mode
              SPI_CR1_BR_2 |
              SPI_CR1_BR_1 |
              SPI_CR1_MSTR | // master mode
              SPI_CR1_CPHA | // cpha=1, cpol=0 for AS5048A
              SPI_CR1_SSM  | // software slave-select mode
              SPI_CR1_SSI  |
              SPI_CR1_SPE;

  enc_poll(); // first one will be garbage
  enc_poll(); // and the second one
}

/*
 * Blocking call to get encoder data for all 3 encoders
 */
void enc_poll()
{
  GPIOE->BSRRH = 1 << PORTE_ENC_CS; // assert (pull down) CS
  for (volatile int i = 0; i < 10; i++) { } // needs at least 350 ns
  SPI4->DR; // clear the rx data register in case it has some garbage
  for (int i = 0; i < NUM_ENC; i++)
  {
    SPI4->DR = 0xffff;
    while (!(SPI4->SR & SPI_SR_TXE)) { } // Wait for transmit buffer empty
    while (!(SPI4->SR & SPI_SR_RXNE)) { } // Wait for recieve buffer not empty
    g_state.encoders[NUM_ENC-1-i] = SPI4->DR & 0x3fff;  // Idx goes 2, 1, 0
  }
  for (volatile int i = 0; i < 1; i++) { } // needs at least 50 ns
  GPIOE->BSRRL = 1 << PORTE_ENC_CS; // de-assert (pull up) CS
  /*
  printf("       enc: %06d  %06d  %06d\r\n",
         g_state.encoders[0],
         g_state.encoders[1],
         g_state.encoders[2]);
  */
}


void enc_poll_nonblocking_tick(const uint8_t bogus __attribute__((unused)))
{
  static uint_fast8_t enc_poll_state_word_idx = 0;
  static uint32_t enc_poll_state_start_time_us = 0;
  static uint8_t all_the_same = TRUE;
  static uint8_t all_the_same_count = 0;
  static uint8_t same_count[3] = {0};
  switch(enc_poll_state)
  {
    case EPS_DONE: // this is the start state
      GPIOE->BSRRH = 1 << PORTE_ENC_CS; // assert (pull down) CS
      enc_poll_state_start_time_us = SYSTIME;
      enc_poll_state = EPS_CS_ASSERTED;
      break;
    case EPS_CS_ASSERTED:
      if (SYSTIME - enc_poll_state_start_time_us > 2)
      {
        all_the_same = TRUE;
        SPI4->DR; // clear the rx data register in case it has some garbage
        enc_poll_state_word_idx = 0;
        SPI4->DR = 0xffff;
        enc_poll_state = EPS_SPI_TXRX;
      }
      break;
    case EPS_SPI_TXRX:
      if ((SPI4->SR & SPI_SR_TXE) && (SPI4->SR & SPI_SR_RXNE))
      {
        uint16_t readValue =  SPI4->DR & 0x3fff;

        if (readValue == g_state.encoders[NUM_ENC-1-enc_poll_state_word_idx]) {
          same_count[NUM_ENC-1-enc_poll_state_word_idx]++;
        } else {
          same_count[NUM_ENC-1-enc_poll_state_word_idx] = 0;
          all_the_same = FALSE;
        }

        g_state.encoders[NUM_ENC-1-(enc_poll_state_word_idx++)] = readValue;
        if (enc_poll_state_word_idx >= NUM_ENC)
        {
          enc_poll_state = EPS_SPI_TXRX_DONE;
          enc_poll_state_start_time_us = SYSTIME;
        }
        else {
          SPI4->DR = 0xffff;
        }
      }
      break;
    case EPS_SPI_TXRX_DONE:
      if (SYSTIME - enc_poll_state_start_time_us > 2)
      {
        if (all_the_same) {
          if (all_the_same_count < LIGHT_LED_THRESHOLD) {
            all_the_same_count++;
          }
        } else {
          all_the_same_count = 0;
        }
        if (all_the_same_count >= LIGHT_LED_THRESHOLD) {
          err_set(ERR_ENC_ALL_STUCK);
        } else {
          err_unset(ERR_ENC_ALL_STUCK);
        }

        if (same_count[0] >= LIGHT_LED_THRESHOLD) {
          err_set(ERR_ENC_1_STUCK);
        } else {
          err_unset(ERR_ENC_1_STUCK);
        }
        if (same_count[1] >= LIGHT_LED_THRESHOLD) {
          err_set(ERR_ENC_2_STUCK);
        } else {
          err_unset(ERR_ENC_2_STUCK);
        }
        if (same_count[2] >= LIGHT_LED_THRESHOLD) {
          err_set(ERR_ENC_3_STUCK);
        } else {
          err_unset(ERR_ENC_3_STUCK);
        }


        GPIOE->BSRRL = 1 << PORTE_ENC_CS; // de-assert (pull up) CS
        enc_poll_state = EPS_DONE;
      }
      break;
    default:
      enc_poll_state = EPS_DONE; // shouldn't get here
      break;
  }
}

