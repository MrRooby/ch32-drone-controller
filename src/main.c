#include <stdint.h>
#include <stdbool.h>
#define SSD1306_128X32

#include "ch32fun.h"
#include <stdio.h>
#include "nrf24l01.h"
#include "ssd1306_i2c.h"
#include "ssd1306.h"
#include "transmitter.h"
#include "serial.h"
#include "timing.h"
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////* CONST DEFINES*///////////////////////////////////////
#define AUX1_PIN 0
#define AUX2_PIN 0
#define AUX1_PORT GPIOC
#define AUX2_PORT GPIOD
#define LOOP_TIME_MS 0
#define OLED_REFRESH_MS 50
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////* STRUCTURE DEFINITIONS*////////////////////////////////////
typedef struct{
  uint8_t throttle;
  uint8_t yaw;
  uint8_t pitch;
  uint8_t roll;
  uint8_t AUX1;
  uint8_t AUX2;
} MyData;

typedef enum {
  L_X     = 0,
  L_Y     = 1,
  R_X     = 3,
  R_Y     = 4,
  BAT_VOL = 7

} ADC_CHANNELS;
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////* FUNCTION DECLARATION *////////////////////////////////////
void resetData(void);
void adc_init(void);
void oled_init(void);
void switches_init(void);
uint16_t adc_get(const ADC_CHANNELS channel);
uint8_t mapJoystickValues(const int val, const int lower, const int middle, const int upper, const uint8_t reverse);
void printDebugToSerial(const MyData *pdata);
void oledBatteryVoltage(void);
void updateData(MyData *pdata);
void oled_refresh_dma(void);
void init_i2c_dma(void);
/////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////* GLOBAL VARIABLES */////////////////////////////////////
MyData data;
extern uint8_t datapipe_address[6][5];
uint8_t pipeOut[5] = {0xE8, 0xE8, 0xF0, 0xF0, 0xE1};
uint32_t showtime = 0;
uint32_t start = 0;
uint32_t last_oled_time = 0;
/////////////////////////////////////////////////////////////////////////////////////////////


int main(void) {
  SystemInit();
  UART_Init(115200);
  printf("Starting CH32V003 NRF24 Controller...\n");
  
  adc_init();
  resetData();
  init_i2c_dma();

  nrf24_device(TRANSMITTER, RESET);
  nrf24_rf_power(0); //Max power

  oled_init();

  // Override the pipe out address to match Arduino code: 0xE8E8F0F0E1
  for(int i=0; i<5; i++) {
    datapipe_address[0][i] = pipeOut[i];
  }

  while(1) {
    uint32_t start = millis();

    updateData(&data);

    if(millis() - last_oled_time > OLED_REFRESH_MS){
      if (DMA1_Channel6->CNTR == 0) {
        last_oled_time = millis();
        oledBatteryVoltage(); // Przygotowuje bufor
        oled_refresh_dma();   // Wysyła w tle
      }
    }

    nrf24_transmit((uint8_t*)&data, sizeof(MyData), NO_ACK_MODE);

    while(millis() - start < LOOP_TIME_MS);
    uint32_t elapsed = millis() - start;
    printf("Elapsed time %lu\n\r", elapsed);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////* FUNCTION DEFINITIONS *////////////////////////////////////
void adc_init(void) {
  // ADCCLK = 24MHz / 2 = 12MHz
  RCC->CFGR0 &= ~(0x1F<<11); // Clear ADC prescaler bits in Clock Configuration Register 0
  RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD | RCC_APB2Periph_ADC1; // Enable Port A, D and ADC1 clocks

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

uint16_t adc_get(const ADC_CHANNELS channel) {
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

uint8_t mapJoystickValues(int val, const int lower, const int middle, const int upper, const uint8_t reverse){
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

void resetData(void) {
  data.throttle = 0;
  data.yaw = 127;
  data.pitch = 127;
  data.roll = 127;
  data.AUX1 = 0;
  data.AUX2 = 0;
}

void switches_init(void){
  AUX1_PORT->CFGLR &= ~(0xff<<(4*1)); // Clear 8 bits (2 nibbles for PD1 and PD2) in the configuration register
  AUX2_PORT->CFGLR &= ~(0xff<<(4*1)); // Clear 8 bits (2 nibbles for PD1 and PD2) in the configuration register
  AUX1_PORT->CFGLR |= (GPIO_CNF_IN_PUPD)<<(4*AUX1_PIN);
  AUX2_PORT->CFGLR |= (GPIO_CNF_IN_PUPD)<<(4*AUX2_PIN);
  AUX1_PORT->OUTDR |= (1<<AUX1_PIN);
  AUX2_PORT->OUTDR |= (1<<AUX2_PIN);
}

void oled_init(void){
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
}

void printDebugToSerial(const MyData *pdata){
  printf("===  DATA VALUES   ===\n");
  printf("Throttle: %d\n", pdata->throttle);
  printf("Yaw: %d\n",      pdata->yaw);
  printf("Pitch: %d\n",    pdata->pitch);
  printf("Roll: %d\n",     pdata->roll);
  printf("======================\n");
}

void updateData(MyData *pdata){
  pdata->throttle = mapJoystickValues( adc_get(L_Y), 13, 524, 1015, 1 );
  pdata->yaw      = mapJoystickValues( adc_get(L_X), 50, 505, 1020, 1 );
  pdata->pitch    = mapJoystickValues( adc_get(R_Y), 12, 544, 1021, 1 );
  pdata->roll     = mapJoystickValues( adc_get(R_X), 34, 522, 1020, 1 );
  pdata->AUX1     = (AUX1_PORT->INDR & (1<<AUX1_PIN)) ? 1 : 0; // Read Port D Input Data Register for pin PD1
  pdata->AUX2     = (AUX2_PORT->INDR & (1<<AUX2_PIN)) ? 1 : 0; // Read Port D Input Data Register for pin PD2
}

void oledBatteryVoltage(void){
  // Voltage calculation (placeholder ADC channel 0 / A0 / PA2)
  uint16_t vdiv_raw = adc_get(BAT_VOL); // Assuming A0 is used for voltage divider
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
}

// Poprawiona inicjalizacja - dopasowana do nazw w ch32fun
void init_i2c_dma(void) {
    RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;
    
    DMA1_Channel6->PADDR = (uint32_t)&I2C1->DATAR;
    DMA1_Channel6->MADDR = (uint32_t)ssd1306_buffer;
    // W ch32fun używamy nazw z cyframi dla konkretnych bitów, jeśli ogólne nie są zdefiniowane
    DMA1_Channel6->CFGR = DMA_CFGR3_DIR | DMA_CFGR1_MINC | DMA_CFGR2_PL_1;
    
    I2C1->CTLR2 |= I2C_CTLR2_DMAEN;
}

// Funkcja odświeżania bez argumentów (używa globalnego bufora biblioteki)
void oled_refresh_dma(void) {
    // Czekaj na wolną magistralę
    while(I2C1->STAR2 & I2C_STAR2_BUSY);
    
    I2C1->CTLR1 |= I2C_CTLR1_START;
    while(!(I2C1->STAR1 & I2C_STAR1_SB));
    
    I2C1->DATAR = (0x3C << 1);
    while(!(I2C1->STAR1 & I2C_STAR1_ADDR));
    (void)I2C1->STAR1; (void)I2C1->STAR2; // Czyszczenie flagi ADDR

    I2C1->DATAR = 0x40; // Control byte: data stream
    while(!(I2C1->STAR1 & I2C_STAR1_BTF));

    // Konfiguracja i start DMA
    DMA1_Channel6->CFGR &= ~DMA_CFGR1_EN;
    DMA1_Channel6->CNTR = sizeof(ssd1306_buffer);
    DMA1_Channel6->MADDR = (uint32_t)ssd1306_buffer;
    DMA1_Channel6->CFGR |= DMA_CFGR1_EN;
}
