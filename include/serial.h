#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>

void UART_Init(uint32_t baudrate);
int putchar(int c);

#endif
