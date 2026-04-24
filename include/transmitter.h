#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#include <stdint.h>

void SPI_Initializer(void);
void pinout_Initializer(void);
uint8_t SPI_send_command(uint8_t command);
void nrf24_CE(uint8_t input);
void nrf24_SPI(uint8_t input);
void delay_function(uint32_t duration_ms);

#endif // TRANSMITTER_H