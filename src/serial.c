#include "ch32fun.h"
#include "serial.h"

void UART_Init(uint32_t baudrate) {
  // Enable GPIOD and USART1 clocks
  RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1;

  // Configure PD5 as Alternate Function Push-Pull (TX)
  GPIOD->CFGLR &= ~(0xf << (4 * 5));
  GPIOD->CFGLR |= (GPIO_Speed_50MHz | GPIO_CNF_OUT_PP_AF) << (4 * 5);

  // USART1 configuration
  USART1->CTLR1 = USART_WordLength_8b | USART_Parity_No | USART_Mode_Tx;
  USART1->CTLR2 = USART_StopBits_1;
  USART1->CTLR3 = USART_HardwareFlowControl_None;

  // Calculate baud rate (ch32v003fun defaults to 48MHz core clock)
#ifndef FUNCONF_SYSTEM_CORE_CLOCK
#define FUNCONF_SYSTEM_CORE_CLOCK 48000000
#endif
  USART1->BRR = FUNCONF_SYSTEM_CORE_CLOCK / baudrate;

  // Enable USART1
  USART1->CTLR1 |= CTLR1_UE_Set;
}

// Override standard putchar to redirect printf
int putchar(int c) {
  if ((char)c == '\n') {
    while (!(USART1->STATR & USART_FLAG_TC));
    USART1->DATAR = '\r';
  }
  while (!(USART1->STATR & USART_FLAG_TC));
  USART1->DATAR = c;
  return c;
}

// Optionally override _write in case newlib stubs are used directly
int _write(int fd, const char *ptr, int len) {
  int i;
  for (i = 0; i < len; i++) {
    putchar(ptr[i]);
  }
  return len;
}
