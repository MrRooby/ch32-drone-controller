#include "ch32fun.h"
#include "transmitter.h"
#include "nrf24l01.h"

// Define pins for NRF24
#define CE_PIN  0 // PD0
#define CSN_PIN 3 // PC3

void SPI_Initializer(void) {
    // Enable GPIOC and SPI1 clocks
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOC | RCC_APB2Periph_SPI1; // Enable APB2 peripheral clock for GPIOC and SPI1
    
    // PC5 - SCK, PC6 - MISO, PC7 - MOSI
    GPIOC->CFGLR &= ~(0xf<<(4*5)); // Clear config bits for PC5
    GPIOC->CFGLR |= (GPIO_Speed_50MHz | GPIO_CNF_OUT_PP_AF)<<(4*5); // Set PC5 to 50MHz Alternate Function Push-Pull (SCK)
    
    GPIOC->CFGLR &= ~(0xf<<(4*7)); // Clear config bits for PC7
    GPIOC->CFGLR |= (GPIO_Speed_50MHz | GPIO_CNF_OUT_PP_AF)<<(4*7); // Set PC7 to 50MHz Alternate Function Push-Pull (MOSI)
    
    GPIOC->CFGLR &= ~(0xf<<(4*6)); // Clear config bits for PC6
    GPIOC->CFGLR |= (GPIO_CNF_IN_FLOATING)<<(4*6); // Set PC6 to Floating Input (MISO)
    
    SPI1->CTLR1 = SPI_Mode_Master | SPI_Direction_2Lines_FullDuplex | SPI_DataSize_8b | 
                  SPI_CPOL_Low | SPI_CPHA_1Edge | SPI_NSS_Soft | SPI_BaudRatePrescaler_16 | SPI_FirstBit_MSB; // Configure SPI1 modes and settings
    
    SPI1->CTLR1 |= CTLR1_SPE_Set; // Enable SPI peripheral
}

void pinout_Initializer(void) {
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD; // Enable APB2 peripheral clock for GPIOC and GPIOD

    // Configure CE (PD0)
    GPIOD->CFGLR &= ~(0xf<<(4*CE_PIN)); // Clear config bits for CE_PIN (PD0)
    GPIOD->CFGLR |= (GPIO_Speed_50MHz | GPIO_CNF_OUT_PP)<<(4*CE_PIN); // Set CE_PIN to 50MHz General Purpose Output Push-Pull
    
    // Configure CSN (PC3)
    GPIOC->CFGLR &= ~(0xf<<(4*CSN_PIN)); // Clear config bits for CSN_PIN (PC3)
    GPIOC->CFGLR |= (GPIO_Speed_50MHz | GPIO_CNF_OUT_PP)<<(4*CSN_PIN); // Set CSN_PIN to 50MHz General Purpose Output Push-Pull
    
    GPIOC->BSHR = (1<<CSN_PIN); // Set CSN High using Bit Set/Reset Register
    GPIOD->BCR = (1<<CE_PIN);   // Set CE Low using Bit Clear Register
}

uint8_t SPI_send_command(uint8_t command) {
    while(!(SPI1->STATR & SPI_I2S_FLAG_TXE)); // Wait until Transmit Data Register is empty
    SPI1->DATAR = command; // Write command to Data Register
    while(!(SPI1->STATR & SPI_I2S_FLAG_RXNE)); // Wait until Receive Data Register is not empty
    return SPI1->DATAR; // Return received data from Data Register
}

void nrf24_CE(uint8_t input) {
    if (input == CE_ON) GPIOD->BSHR = (1<<CE_PIN); // Set CE pin High in Bit Set/Reset Register
    else GPIOD->BCR = (1<<CE_PIN); // Set CE pin Low in Bit Clear Register
}

void nrf24_SPI(uint8_t input) {
    if (input == SPI_ON) GPIOC->BCR = (1<<CSN_PIN); // Set CSN pin Low in Bit Clear Register (activate SPI)
    else GPIOC->BSHR = (1<<CSN_PIN); // Set CSN pin High in Bit Set/Reset Register (deactivate SPI)
}

void delay_function(uint32_t duration_ms){
  Delay_Ms(duration_ms);
}
