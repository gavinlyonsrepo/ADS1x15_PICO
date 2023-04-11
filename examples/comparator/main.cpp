/*
 * File: main.cpp
 * Description:
 * ADC ADS1015 example file for RPI Rp2040 PICO C++ SDK
 * Shows use of comparator function.
 * Description: See URL for full details.
 * URL: https://github.com/gavinlyonsrepo/ADS1x15_PICO
 */

// Library
#include <stdio.h>
#include "pico/stdlib.h"
#include "ads1x15/ads1x15.hpp"

PICO_ADS1015 ads;

// main function
int main(){

  stdio_init_all(); // Initialize chosen serial port, default 38400 baud
  busy_wait_ms(1000);
  printf("ADS1x15\r\n");
  printf("Single-ended readings from AIN0 with comparator\r\n");
  printf("Comp Threshold: 1000 : ALERT pin active low\r\n");
  
  int16_t adc0;
  ads.setGain(GAIN_ONE);

  if (!ads.beginADSX(ADSX_ADDRESS, i2c1, 100, 18,19)) {
    printf("Failed to initialize ADS.\r\n");
    while (1);
  }
  // Setup comparator on channel 0
  ads.startComparator_SingleEnded(0, 1000);

  while (1)
  {
    // Comparator will only de-assert after a read
    adc0 = ads.getLastConversionResults();
    printf("AIN0: %i \r\n", adc0);

    busy_wait_ms(1000);
  }
} //end of main 