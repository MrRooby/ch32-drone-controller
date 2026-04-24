#define DEBUG
#define OLED

#include "ch32fun.h"
#include <stdio.h>
#include "nrf24l01.h"
#ifdef DEBUG
#include "serial.h"
#endif /* ifdef DEBUG */

#ifdef OLED
// Define OLED size and include library
#define SSD1306_128X32
#include "ssd1306_i2c.h"
#include "ssd1306.h"
#endif /* ifdef OLED */

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////* CONST DEFINES*///////////////////////////////////////
#define CE_PIN  0 // PD0
#define CSN_PIN 3 // PC3
/////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////* STRUCTURE DEFINITIONS*////////////////////////////////////
struct MyData {
  uint8_t throttle;
  uint8_t yaw;
  uint8_t pitch;
  uint8_t roll;
  uint8_t AUX1;
  uint8_t AUX2;
};
/////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////* FUNCTION DECLARATION *////////////////////////////////////
void delay_function(uint32_t duration_ms);
void resetData(void);
void SPI_Initializer(void);
void pinout_Initializer(void);
uint8_t SPI_send_command(uint8_t command);
void nrf24_CE(uint8_t input);
void nrf24_SPI(uint8_t input);
void adc_init(void);
uint16_t adc_get(uint8_t channel);
uint8_t mapJoystickValues(int val, int lower, int middle, int upper, uint8_t reverse);
/////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////* GLOBAL VARIABLES */////////////////////////////////////
struct MyData data;
/////////////////////////////////////////////////////////////////////////////////////////////


int main() {
    SystemInit();

#ifdef DEBUG
    UART_Init(115200); // Initialize UART at 115200 baudrate for printf override
    printf("Starting CH32V003 NRF24 Controller...\n");
#endif /* ifdef DEBUG */

    adc_init();
    resetData();
    nrf24_device(TRANSMITTER, RESET);

#ifdef OLED
    if (!ssd1306_i2c_init()) {
        ssd1306_init();
        
        ssd1306_setbuf(0);
        ssd1306_drawstr(8, 0, "RISCy", 1);
        ssd1306_drawstr(0, 16, "Transmitter", 1);
        ssd1306_refresh();
        Delay_Ms(1200);

        ssd1306_setbuf(0);
        ssd1306_drawstr(0, 0, "KOCHAC PAPIEZA", 1);
        ssd1306_drawstr(0, 16, "Mini FPV Drone", 1);
        ssd1306_refresh();
        Delay_Ms(2200);
        
        ssd1306_setbuf(0);
        ssd1306_refresh();
    }
#endif /* ifdef OLED */

    // Override the pipe out address to match Arduino code: 0xE8E8F0F0E1
    extern uint8_t datapipe_address[6][5];
    uint8_t pipeOut[5] = {0xE8, 0xE8, 0xF0, 0xF0, 0xE1};
    for(int i=0; i<5; i++) {
        datapipe_address[0][i] = pipeOut[i];
    }
    
    
    // Configure buttons
    // Using PD1 for AUX1 and PD2 for AUX2 (pull-up)
    GPIOD->CFGLR &= ~(0xff<<(4*1)); // Clear 8 bits (2 nibbles for PD1 and PD2) in the configuration register
    GPIOD->CFGLR |= (GPIO_CNF_IN_PUPD)<<(4*1) | (GPIO_CNF_IN_PUPD)<<(4*2); // Set PD1 and PD2 to input with pull-up/pull-down mode
    GPIOD->OUTDR |= (1<<1) | (1<<2); // Write 1 to PD1 and PD2 output data register to activate pull-up resistors
    
    while(1) {
        // You will need to configure these channel mappings to the exact pins you used:
        // A1 (PA1 - CH1), A2 (PC4 - CH2), A3 (PD2 - CH3), A4 (PD3 - CH4)
        uint16_t throttle_raw = adc_get(3); // Example: PD2 (CH3)
        uint16_t yaw_raw      = adc_get(1); // Example: PA1 (CH1)
        uint16_t pitch_raw    = adc_get(2); // Example: PC4 (CH2)
        uint16_t roll_raw     = adc_get(4); // Example: PD3 (CH4)
        
        data.throttle = mapJoystickValues( throttle_raw, 13, 524, 1015, 1 );
        data.yaw      = mapJoystickValues( yaw_raw,      50, 505, 1020, 1 );
        data.pitch    = mapJoystickValues( pitch_raw,    12, 544, 1021, 1 );
        data.roll     = mapJoystickValues( roll_raw,     34, 522, 1020, 1 );
        
        data.AUX1 = (GPIOD->INDR & (1<<1)) ? 1 : 0; // Read Port D Input Data Register for pin PD1
        data.AUX2 = (GPIOD->INDR & (1<<2)) ? 1 : 0; // Read Port D Input Data Register for pin PD2

#ifdef OLED
        // Voltage calculation (placeholder ADC channel 0 / A0 / PA2)
        uint16_t vdiv_raw = adc_get(0); // Assuming A0 is used for voltage divider
        float vol = ((vdiv_raw / 1024.0) * 10.0);
        
        char vol_str[16];
        // Printf in minimalistic c libraries usually does not support floats. We convert to integer parts:
        int vol_int = (int)vol;
        int vol_frac = (int)((vol - vol_int) * 100);
        snprintf(vol_str, sizeof(vol_str), "%d.%02dV", vol_int, vol_frac);

        ssd1306_setbuf(0);
        ssd1306_drawstr(0, 0, "Voltage:", 1);
        ssd1306_drawstr(0, 16, vol_str, 1);
        ssd1306_refresh();
#endif /* ifdef OLED */

#ifdef DEBUG
        printf("===   RAW VALUES   ===\n");
        printf("Throttle: %d\n", throttle_raw);
        printf("Yaw: %d\n",      yaw_raw);
        printf("Pitch: %d\n",    pitch_raw);
        printf("Roll: %d\n",     roll_raw);
        printf("======================\n");

        printf("===  DATA VALUES   ===\n");
        printf("Throttle: %d\n", data.throttle);
        printf("Yaw: %d\n",      data.yaw);
        printf("Pitch: %d\n",    data.pitch);
        printf("Roll: %d\n",     data.roll);
        printf("======================\n");
#endif /* ifdef DEBUG */

        nrf24_transmit((uint8_t*)&data, sizeof(struct MyData), NO_ACK_MODE);
        
        Delay_Ms(20); // ~50Hz refresh
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////* FUNCTION DEFINITIONS *////////////////////////////////////
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

void adc_init(void) {
    // ADCCLK = 24MHz / 2 = 12MHz
    RCC->CFGR0 &= ~(0x1F<<11); // Clear ADC prescaler bits in Clock Configuration Register 0
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_ADC1; // Enable Port A, C, D and ADC1 clocks
    
    // Reset ADC1
    RCC->APB2PRSTR |= RCC_APB2Periph_ADC1; // Set ADC1 reset bit in APB2 peripheral reset register
    RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1; // Clear ADC1 reset bit
    
    ADC1->RSQR1 = 0; // Clear Regular Sequence Register 1 (setting 1 conversion)
    ADC1->RSQR2 = 0; // Clear Regular Sequence Register 2
    
    // Turn on ADC and enable software triggering
    ADC1->CTLR2 |= ADC_ADON | ADC_EXTSEL; // Set ADON (A/D Converter ON) and EXTSEL (External Event Select) bits in Control Register 2
    
    // Reset Calibration
    ADC1->CTLR2 |= ADC_RSTCAL; // Set Reset Calibration bit
    while(ADC1->CTLR2 & ADC_RSTCAL); // Wait until Reset Calibration bit is cleared by hardware
    
    // Calibrate
    ADC1->CTLR2 |= ADC_CAL; // Set A/D Calibration bit
    while(ADC1->CTLR2 & ADC_CAL); // Wait until A/D Calibration is complete (bit cleared by hardware)
}

uint16_t adc_get(uint8_t channel) {
    ADC1->RSQR3 = channel; // Set the 1st conversion in regular sequence register 3 to the chosen channel
    
    // Set sampling time for channel to longest (239.5 cycles)
    if (channel < 10) {
        ADC1->SAMPTR2 &= ~(ADC_SMP0<<(3*channel)); // Clear sample time bits for the specific channel in Sample time register 2
        ADC1->SAMPTR2 |= 7<<(3*channel);  // Set sample time bits to 111 (239.5 cycles) for the specific channel
    }
    
    ADC1->CTLR2 |= ADC_SWSTART; // Set Start Conversion of regular channels bit inside Control Register 2
    while(!(ADC1->STATR & ADC_EOC)); // Wait until End Of Conversion bit is set in Status Register
    return ADC1->RDATAR; // Return the 16-bit converted data from Regular Data Register
}

uint8_t mapJoystickValues(int val, int lower, int middle, int upper, uint8_t reverse){
  if (val < lower) val = lower;
  if (val > upper) val = upper;
  
  if ( val < middle )
    val = (val - lower) * 128 / (middle - lower);
  else
    val = 128 + (val - middle) * 127 / (upper - middle);
    
  if (val < 0) val = 0;
  if (val > 255) val = 255;
    
  return ( reverse ? 255 - val : val );
}

void resetData() {
  data.throttle = 0;
  data.yaw = 127;
  data.pitch = 127;
  data.roll = 127;
  data.AUX1 = 0;
  data.AUX2 = 0;
}

void delay_function(uint32_t duration_ms){
  Delay_Ms(duration_ms);
}
