#define SSD1306_128X32

#include "ch32fun.h"
#include <stdio.h>
#include "nrf24l01.h"
#include "serial.h"
#include "ssd1306_i2c.h"
#include "ssd1306.h"
#include "transmitter.h"
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////* CONST DEFINES*///////////////////////////////////////
#define ANALOG_L_X 1
#define ANALOG_L_Y 0
#define ANALOG_R_X 3
#define ANALOG_R_Y 4

#define BATT_VOL 2

#define AUX1_PIN 0
#define AUX2_PIN 0
#define AUX1_PORT GPIOC
#define AUX2_PORT GPIOD
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
/////////////////////////////////* FUNCTION DECLARATION *////////////////////////////////////
void resetData(void);
void adc_init(void);
void switches_init(void);
uint16_t adc_get(uint8_t channel);
uint8_t mapJoystickValues(int val, int lower, int middle, int upper, uint8_t reverse);
/////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////* GLOBAL VARIABLES */////////////////////////////////////
struct MyData data;
extern uint8_t datapipe_address[6][5];
uint8_t pipeOut[5] = {0xE8, 0xE8, 0xF0, 0xF0, 0xE1};
/////////////////////////////////////////////////////////////////////////////////////////////


int main() {
  SystemInit();

  UART_Init(115200); // Initialize UART at 115200 baudrate for printf override
  printf("Starting CH32V003 NRF24 Controller...\n");

  adc_init();
  resetData();

  nrf24_device(TRANSMITTER, RESET);

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

  // Override the pipe out address to match Arduino code: 0xE8E8F0F0E1
  for(int i=0; i<5; i++) {
    datapipe_address[0][i] = pipeOut[i];
  }

  while(1) {
    // You will need to configure these channel mappings to the exact pins you used:
    // A1 (PA1 - CH1), A2 (PC4 - CH2), A3 (PD2 - CH3), A4 (PD3 - CH4)
    uint16_t throttle_raw = adc_get(3); // Example: PD2 (CH3)
    uint16_t yaw_raw      = adc_get(1); // Example: PA1 (CH1)
    uint16_t pitch_raw    = adc_get(0); // Example: PC4 (CH2)
    uint16_t roll_raw     = adc_get(4); // Example: PD3 (CH4)

    data.throttle = mapJoystickValues( throttle_raw, 13, 524, 1015, 1 );
    data.yaw      = mapJoystickValues( yaw_raw,      50, 505, 1020, 1 );
    data.pitch    = mapJoystickValues( pitch_raw,    12, 544, 1021, 1 );
    data.roll     = mapJoystickValues( roll_raw,     34, 522, 1020, 1 );

    data.AUX1 = (AUX1_PORT->INDR & (1<<AUX1_PIN)) ? 1 : 0; // Read Port D Input Data Register for pin PD1
    data.AUX2 = (AUX2_PORT->INDR & (1<<AUX2_PIN)) ? 1 : 0; // Read Port D Input Data Register for pin PD2

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

    nrf24_transmit((uint8_t*)&data, sizeof(struct MyData), NO_ACK_MODE);

    Delay_Ms(2000); // ~50Hz refresh
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////* FUNCTION DEFINITIONS *////////////////////////////////////
void adc_init(void) {
  // ADCCLK = 24MHz / 2 = 12MHz
  RCC->CFGR0 &= ~(0x1F<<11); // Clear ADC prescaler bits in Clock Configuration Register 0
  RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD; // Enable Port A, D and ADC1 clocks

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

void switches_init(void){
  AUX1_PORT->CFGLR &= ~(0xff<<(4*1)); // Clear 8 bits (2 nibbles for PD1 and PD2) in the configuration register
  AUX2_PORT->CFGLR &= ~(0xff<<(4*1)); // Clear 8 bits (2 nibbles for PD1 and PD2) in the configuration register
  AUX1_PORT->CFGLR |= (GPIO_CNF_IN_PUPD)<<(4*AUX1_PIN);
  AUX2_PORT->CFGLR |= (GPIO_CNF_IN_PUPD)<<(4*AUX2_PIN);
  AUX1_PORT->OUTDR |= (1<<AUX1_PIN);
  AUX2_PORT->OUTDR |= (1<<AUX2_PIN);
}
