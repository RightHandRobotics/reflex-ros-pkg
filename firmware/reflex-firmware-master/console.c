#include "console.h"

// pin connections
// PE0 = uart8 TX on AF8
// PE1 = uart8 RX on AF8

#define PORTE_RX 0
#define PORTE_TX 1

void consoleInit()
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
  RCC->APB1ENR |= RCC_APB1ENR_UART8EN;
  GPIOE->MODER   |= (0x2) << (PORTE_TX * 2);
  GPIOE->AFR[0]  |= (0x8) << (PORTE_TX * 4);
  // RX not used at the moment. TX only used for stdout
  UART8->CR1 &= ~USART_CR1_UE;
  UART8->CR1 |=  USART_CR1_TE | USART_CR1_RE;
  // we want 1 megabit. do this with mantissa=2 and fraction (sixteenths)=10
  UART8->BRR  = (22 << 4) | 12;//42000000/115200;//(((uint16_t)2) << 4) | 10;  // 10 << 4 | 1010 -> 101000
  UART8->CR1 |=  USART_CR1_UE;
}
// (22 << 4) | 12;  10110 1100 

int consolePrint(const uint8_t *buffer, uint32_t len)
{
  int startTime = SYSTIME;
  
  // make sure transmission buffer is clear
  while (!(UART8->SR & USART_SR_TXE) && (SYSTIME - startTime < UART_TIMEOUT));
  if (SYSTIME - startTime > UART_TIMEOUT)
    return -1;

  // start transmission
  for (uint32_t i = 0; i < len; i++)
  {
    while (!(UART8->SR & USART_SR_TXE) && (SYSTIME - startTime < UART_TIMEOUT)); // wait for tx buffer to clear
    UART8->DR = buffer[i];
    if (SYSTIME - startTime > UART_TIMEOUT)
      return -1;
  }
  // while (!(UART8->SR & USART_SR_TC)); // wait for TX to finish

  // return number of bytes transmitted
  return len;
}

