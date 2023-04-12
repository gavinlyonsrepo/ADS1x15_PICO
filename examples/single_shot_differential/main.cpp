/*
 * File: main.cpp
 * Description:
 * ADC ADS1015 example file for RPI Rp2040 PICO C++ SDK
 * Shows use of non-blocking single shot Differential mode.
 * Description: See URL for full details.
 * URL: https://github.com/gavinlyonsrepo/ADS1x15_PICO
 */

// Libraries
#include <stdio.h>
#include "pico/stdlib.h"
#include "ads1x15/ads1x15.hpp"

PICO_ADS1015 ads; // 12 bit ADS1015 instance

// Main Function
int main() {
  int16_t results = 0;
  float volts =  0.0;

  busy_wait_ms(500);
  stdio_init_all(); // Initialize chosen serial port, default 38400 baud
  busy_wait_ms(1000);
  printf("ADS1x15 : Conversion NON-Interrupt , single-shot!\r\n");
  printf("Getting differential reading from AIN0 (P) and AIN1 (N)\r\n");

  ads.setGain(ADSXGain_ONE);

  if (!ads.beginADSX(ADSX_ADDRESS_GND, i2c1, 100, 18,19)) {
    printf("Failed to initialize ADS.\r\n");
    while (1);
  }

  // Start the first conversion. single shot
  ads.startADCReading(ADSXRegConfigMuxDiff_0_1, ADSSingleShotMode);

  while(1)
  {
    if (ads.conversionComplete() == true)
    {
      results = ads.getLastConversionResults();
      volts =  ads.computeVolts(results);

      printf("Differential: %i  %f  V\n", results , volts); 

      // Start another conversion, single shot
      ads.startADCReading(ADSXRegConfigMuxDiff_0_1, ADSSingleShotMode);
      busy_wait_ms(2500);
    }
   
  }
  
}