/*!
 * @file main.cpp
 * @brief
 * ADC ADS1015 example file for RPI Rp2040 PICO C++ SDK
 * Shows use of non-blocking single shot Differential mode.
 * URL: https://github.com/gavinlyonsrepo/ADS1x15_PICO
 */

// Libraries
#include <stdio.h>
#include "pico/stdlib.h"
#include "ads1x15/ads1x15.hpp"

PICO_ADS1015 ads; // 12 bit ADS1015 instance
uint16_t I2CSpeed = 100;		 // I2C speed in Khz
uint8_t DataGPIO = 18;			 // I2C GPIO for data line
uint8_t ClockGPIO = 19;			 // I2C GPIO for Clock line
uint32_t I2CTimeout = 50000; // I2C timeout delay in uS.

// Main Function
int main() {
  int16_t results = 0;
  float volts =  0.0;

  busy_wait_ms(500);
  stdio_init_all(); // Initialize chosen serial port, default 38400 baud
  busy_wait_ms(1000);
  printf("ADS1x15 : Conversion NON-Interrupt , single-shot!\r\n");
  printf("Getting differential reading from AIN0 (P) and AIN1 (N)\r\n");

  ads.setGain(ads.ADSXGain_ONE);

  if (!ads.beginADSX(ads.ADSX_ADDRESS_GND, i2c1, I2CSpeed, DataGPIO, ClockGPIO, I2CTimeout)) {
    printf("Failed to initialize ADS.\r\n");
    while (1);
  }

  // Start the first conversion. single shot
  ads.startADCReading(ads.ADSXRegConfigMuxDiff_0_1, ads.ADSSingleShotMode);

  while(1)
  {
    if (ads.conversionComplete() == true)
    {
      results = ads.getLastConversionResults();
      volts =  ads.computeVolts(results);

      printf("Differential: %i  %f  V\n", results , volts); 

      // Start another conversion, single shot
      ads.startADCReading(ads.ADSXRegConfigMuxDiff_0_1, ads.ADSSingleShotMode);
      busy_wait_ms(2500);
    }
   
  }
  
}