/*
 * File: main.cpp
 * Description:
 * ADC ADS1015  example file for RPI Rp2040 PICO C++ SDK
 * Shows use of Differential mode, from AIN0 (P) and AIN1 (N).
 * Description: See URL for full details.
 * URL: https://github.com/gavinlyonsrepo/ADS1x15_PICO
 */

// Library
#include <stdio.h>
#include "pico/stdlib.h"
#include "ads1x15/ads1x15.hpp"

PICO_ADS1015 ads;

// Main function
int main  ()
{
  stdio_init_all(); // Initialize chosen serial port, default 38400 baud
  busy_wait_ms(1000);
  printf("ADS1x15\r\n");
  printf("Getting differential reading from AIN0 (P) and AIN1 (N)\r\n");

  int16_t results = 0;

  // Update this value based on the IC and the gain settings! 
  // In this case ADS1015 ADSXGain_ONE = 2mV aka 2.0 multiplier see README
  float multiplier = 2.0F;  // ADS1015 ADSXGain_ONE (12-bit results) 
  ads.setGain(ADSXGain_ONE);

  if (!ads.beginADSX(ADSX_ADDRESS_GND, i2c1, 100, 18,19)) {
    printf("Failed to initialize ADS.\r\n");
    while (1);
  }

  while(1)
  {
    results = ads.readADC_Diff01();
    printf("Differential :  %f mV \r\n", results * multiplier);
    busy_wait_ms(2500);
  }
} //end of main