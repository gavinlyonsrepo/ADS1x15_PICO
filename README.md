[![Website](https://img.shields.io/badge/Website-Link-blue.svg)](https://gavinlyonsrepo.github.io/)  [![Rss](https://img.shields.io/badge/Subscribe-RSS-yellow.svg)](https://gavinlyonsrepo.github.io//feed.xml)  [![Donate](https://img.shields.io/badge/Donate-PayPal-green.svg)](https://www.paypal.com/paypalme/whitelight976)

# ADS1x15_PICO

![image](https://github.com/gavinlyonsrepo/ADS1x15_PICO/blob/main/extra/doc/images/ads.jpg)

## Table of contents

  * [Overview](#overview)
  * [Examples](#examples)
  * [Software](#software)
    * [Constructor](#constructor)
    * [I2C settings](#i2c-settings)
    * [Gain settings](#gain-settings)
    * [Data rate settings](#data-rate-settings)
    * [Modes](#modes)
  * [Hardware](#hardware)

## Overview

* Name: ADS1x15_PICO
* Description: 

Library Driver for  ADC sensor,  ADS1015 and ADS1115 modules,
for Raspberry pi PICO RP2040. 

ADC Ultra-Small, Low-Power, I2C-Compatible, With Internal Reference, Oscillator, and Programmable Comparator.
1. ADS1015  3.3-kSPS, 12-Bit ADCs
2. ADS1115  860-SPS,  16-Bit ADCs 

* Toolchain
	1. Raspberry pi PICO RP2040
	2. SDK C++, compiler G++ for arm-none-eabi
	3. CMAKE , VScode

* Only tested at present on ADS1015 12 bit device, should work on ADS1115 as well
* By adjusting the I2C address you can add 4 of these devices on the I2C bus.
* Lets the user add 4 more ADC channels to a PICO RP2040

## Examples


There are five example files.
The example files are in example folder. To build the one you want, edit the CMakeLists.txt file add_executable(${PROJECT_NAME} section, comment in one example file path and one only. 

| Path Name | Function |
| --- | --- |
| single_ended  | Shows single ended basic use case, Reading ADC values from AIN0 to AIN3 |
| continuous_interrupt_differential | Shows a differential reading in continuous conversion mode with interrupt |
| single_shot_differential | Shows a differential reading in single shot mode |
| differential | Shows a differential reading between AIN1 P and AIN0 N |
| comparator | Shows  use of comparator threshold mode in default settings |

The example files output Data to the PC serial port using printf,  default settings.(baud rate 38400). See "modes" section below for more information.

## Software


### Constructor

There are two different constructors one for ADS1015 (12 bit) and one for ADS1115 (16 bit)
All examples files are setup for ADS1015. ADS1115 not tested but should work.

### I2C settings

In the beginADSX method the user can pass in a number of I2C related arguments.
In the header file it is also possible to turn on I2C debugging messages and
adjust the timeout of the I2C functions if necessary.

| Function | Default |
| --- |  --- | 
| I2C address 8-bit,  This is affected by Addr line on chip, see 8.5.1.1 datasheet | 0x48 |
| I2C instance of port IC20 or I2C1 | I2C1 |
| I2C Clk speed mode in khz, see 8.5.1.3 datasheet | 100 |
| I2C data line | GPIO 18 |
| I2C clock Line | GPIO 19 |

### Gain settings

The ADC input range or gain can be changed  with setGain()
method, but NEVER exceed VDD +0.3V max, or to
exceed the upper and lower limits if you adjust the input range.
Setting these values incorrectly may damage the ADC module.
The default if setGain() not called is ADSXGain_TWOTHIRDS. 
All examples files are set to ADSXGain_ONE. See datasheet 8.3.3
Programmable gain amplifier(PGA settings) 

| Argument to setGain(arg) | Voltage range | ADS1015 Res.  per 1 bit  |  ADS1115  Res. per  1 bit |
| --- | --- | --- | --- |
| ADSXGain_TWOTHIRDS | 2/3 gain +/- 6.144V | 3mV | 0.1875mV |
| ADSXGain_ONE | 1x gain +/- 4.096V | 2mV | 0.125mV |
| ADSXGain_TWO | 2x gain +/- 2.048V| 1mV | 0.0625mV |
| ADSXGain_FOUR | 4x gain +/- 1.024V | 0.5mV | 0.03125mV |
| ADSXGain_EIGHT | 8x gain +/- 0.512V | 0.25mV | 0.015625mV |
| ADSXGain_SIXTEEN | 16x gain +/- 0.256V  | 0.125mV | 0.0078125mV |

### Data rate settings
Data rate can also be set(method setDataRate). See "Data rate" section in header files for all values. Default(mid range) is ADS1015 = 1600 SPS,  ADS1115 = 128 SPS.

### Modes

Operational mode , the ADS sensor can operate in single shot or continuous mode.
Depending on how often conversions needed you can tune the mode.

In single shot mode the ADS1X15 enter a power-down state between conversions to save power.
The devices are automatically powered down after one conversion
in single-shot mode. therefore, power consumption is
significantly reduced during idle periods.

In continuous-conversion mode (MODE bit set to 0), the ADS1X15 perform conversions continuously. It outputs an 8uS pulse on the ALERT line when ready. This can be used to trigger an interrupt to tell PICO to collect the ready data.
![image](https://github.com/gavinlyonsrepo/ADS1x15_PICO/blob/main/extra/doc/images/3.jpg)

Differential. The differential reading of the ADC can also be done with asynchronous calls measuring the difference in voltage between two channels.

Single ended. Four single-ended input measurements Can be performed on the 4 channels.

Comparator Mode. The ADS sensor feature a programmable digital comparator that can issue an alert on the ALERT pin when a threshold is passed. The settings are set to default can be changed within the "startComparator_SingleEnded" method.

## Hardware

**Pinout**

| Pin | Function | 
| --- | --- |
| A3 | AIN3  Analog Input Channel 3| 
| A2 | AIN3  Analog Input Channel 2| 
| A1 | AIN3  Analog Input Channel 1| 
| A0 | AIN3  Analog Input Channel 0| 
| ALERT | Digital output Comparator output or conversion ready  | 
| ADDR | Digital Input I2C slave address select , see 8.5.1.1 datasheet|
| SDA |Digital I/O Serial data. Transmits and receives data |
| SCL | Digital input Serial clock input. Clocks data on SDA |
| GND | Ground |
| VCC  | Power supply 2-5V in PICO case connect to 3.3v|

![image 3](https://github.com/gavinlyonsrepo/ADS1x15_PICO/blob/main/extra/doc/images/hw.jpg)
