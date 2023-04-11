/*
 * File: main.cpp
 * Description:
 * ADC ADS1015 example file for RPI Rp2040 PICO C++ SDK
 * Shows use of Continuous Differential mode using an interrupt from ALERT pin.
 * Description: See URL for full details.
 * URL: https://github.com/gavinlyonsrepo/ADS1x15_PICO
 */

// Library
#include <stdio.h>
#include "pico/stdlib.h"
#include "ads1x15/ads1x15.hpp"

PICO_ADS1015 ads; // 12 bit ADS1015

// Interrupt related parameters
volatile bool newData = false;
void gpio_callback(uint gpio, uint32_t events) {  newData = true; }
// Pin connected to the ALERT/RDY signal for new sample notification.
constexpr int READY_PIN = 22;


// Main loop
int main () {
  int16_t results = 0;
  float volts =  0.0;

  busy_wait_ms(500);
  stdio_init_all(); // Initialize chosen serial port, default 38400 baud
  busy_wait_ms(1000);
  printf("ADS1x15 : Start continuous-conversion Interrupt!\r\n");
  printf("Getting differential reading from AIN0 (P) and AIN1 (N)\r\n");

  // set up GPIO ready pin on Alert/RDY pin
  gpio_init(READY_PIN);
  gpio_set_dir(READY_PIN, GPIO_IN);
  gpio_pull_up(READY_PIN);

  ads.setGain(GAIN_ONE);

  if (!ads.beginADSX(ADSX_ADDRESS, i2c1, 100, 18,19)) {
    printf("ADS1x15 : Failed to initialize ADS.!\r\n");
    while (1);
  }

  // We get a falling edge every time a new sample is ready.
  gpio_set_irq_enabled_with_callback(READY_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

  // Start continuous conversions.
  ads.startADCReading(ADSX_REG_CONFIG_MUX_DIFF_0_1, ADSContinuousMode);

  while(1) // loop here forever
  {
    if (newData == true) // will be set to true by interrupt
    {
      results = ads.getLastConversionResults();
      volts =  ads.computeVolts(results);
      printf("Differential: %i %f  V\n", results , volts); 
      newData = false;
    }
    // prevent serial port flooding with data
    busy_wait_ms(2500);
  }
} //   End of main