/*
 * File: main.cpp
 * Description:
 * ADC ADS1015 example file for RPI Rp2040 PICO C++ SDK
 * Shows use of single ended ADC mode.
 * Description: See URL for full details.
 * URL: https://github.com/gavinlyonsrepo/ADS1x15_PICO
 */

// Libraries
#include <stdio.h>
#include "pico/stdlib.h"
#include "ads1x15/ads1x15.hpp"

PICO_ADS1015 ads; // 12 bit ADS1015 instance

// Main function
int main() {

  // Setup
  busy_wait_ms(500);
  stdio_init_all(); // Initialize chosen serial port, default 38400 baud
  busy_wait_ms(1000);
  printf("ADS1x15 : Start Single End example.\r\n");
  printf("Getting Single End readings from AIN0-3");

  int16_t adc0, adc1, adc2, adc3;
  float volts0, volts1, volts2, volts3;

  ads.setGain(ADSXGain_ONE);

  if (!ads.beginADSX(ADSX_ADDRESS_GND, i2c1, 100, 18,19)) {
    printf("ADS1x15 : Failed to initialize ADS.!\r\n");
    while (1);
  }

  // Forever Loop
  while(1)
  {
    adc0 = ads.readADC_SingleEnded(ADSX_AIN0);
    adc1 = ads.readADC_SingleEnded(ADSX_AIN1);
    adc2 = ads.readADC_SingleEnded(ADSX_AIN2);
    adc3 = ads.readADC_SingleEnded(ADSX_AIN3);

    volts0 = ads.computeVolts(adc0);
    volts1 = ads.computeVolts(adc1);
    volts2 = ads.computeVolts(adc2);
    volts3 = ads.computeVolts(adc3);

    printf("------------------------------\r\n");
    printf("AIN0: %i  %f V \r\n", adc0, volts0);
    printf("AIN1: %i  %f V \r\n", adc1, volts1);
    printf("AIN2: %i  %f V \r\n", adc2, volts2);
    printf("AIN3: %i  %f V \r\n", adc3, volts3);
    
    busy_wait_ms(2500);
  }

} // end of main function