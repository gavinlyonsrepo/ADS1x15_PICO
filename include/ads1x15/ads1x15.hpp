/*!
 * @file ads1x15.hpp
 * @brief
 * 	ADC ADS1015 ADS1115 library hpp file for RPI Rp2040 PICO C++ SDK
 * 	URL: https://github.com/gavinlyonsrepo/ADS1x15_PICO
 */

#ifndef __ADS1X15LIB__
#define __ADS1X15LIB__

#include <stdio.h> // optional for printf debug error messages
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// comment this in for serial debugging
// #define ADS_SERIAL_DEBUG 1

/*! Super class for all varients ADS1x15 */
class PICO_ADS1X15
{
	protected:
	struct ADSX
	{
		// Register Pointers
		static constexpr uint16_t REG_POINTER_MASK = 0x03;			/**< Register pointer mask */
		static constexpr uint16_t REG_POINTER_CONVERT = 0x00;		/**< Conversion register */
		static constexpr uint16_t REG_POINTER_CONFIG = 0x01;		/**< Configuration register */
		static constexpr uint16_t REG_POINTER_LOWTHRESH = 0x02; /**< Low threshold register */
		static constexpr uint16_t REG_POINTER_HITHRESH = 0x03;	/**< High threshold register */

		static constexpr uint16_t REG_CONFIG_OS_MASK = 0x8000;		/**< OS Mask */
		static constexpr uint16_t REG_CONFIG_OS_SINGLE = 0x8000;	/**< Write: Set to start a single conversion */
		static constexpr uint16_t REG_CONFIG_OS_BUSY = 0x0000;		/**< R/W: Bit=0 when conversion is in progress */
		static constexpr uint16_t REG_CONFIG_OS_NOTBUSY = 0x8000; /**< R/W: Bit=1 when device is not performing a conversion */
		static constexpr uint16_t REG_CONFIG_MUX_MASK = 0x7000;		/**< Mux Mask */
		// Register Configs
		static constexpr uint16_t REG_CONFIG_PGA_MASK = 0x0E00;		/**< PGA Mask */
		static constexpr uint16_t REG_CONFIG_PGA_6_144V = 0x0000; /**< +/-6.144V range = Gain 2/3 */
		static constexpr uint16_t REG_CONFIG_PGA_4_096V = 0x0200; /**< +/-4.096V range = Gain 1 */
		static constexpr uint16_t REG_CONFIG_PGA_2_048V = 0x0400; /**< +/-2.048V range = Gain 2 (default) */
		static constexpr uint16_t REG_CONFIG_PGA_1_024V = 0x0600; /**< +/-1.024V range = Gain 4 */
		static constexpr uint16_t REG_CONFIG_PGA_0_512V = 0x0800; /**< +/-0.512V range = Gain 8 */
		static constexpr uint16_t REG_CONFIG_PGA_0_256V = 0x0A00; /**< +/-0.256V range = Gain 16 */

		static constexpr uint16_t REG_CONFIG_MODE_MASK = 0x0100;	 /**< Mode Mask */
		static constexpr uint16_t REG_CONFIG_MODE_CONTIN = 0x0000; /**< Continuous conversion mode */
		static constexpr uint16_t REG_CONFIG_MODE_SINGLE = 0x0100; /**< Power-down single-shot mode (default) */

		static constexpr uint16_t REG_CONFIG_RATE_MASK = 0x00E0;		/**< Data Rate Mask */
		static constexpr uint16_t REG_CONFIG_CMODE_MASK = 0x0010;		/**< CMode Mask */
		static constexpr uint16_t REG_CONFIG_CMODE_TRAD = 0x0000;		/**< Traditional comparator with hysteresis (default) */
		static constexpr uint16_t REG_CONFIG_CMODE_WINDOW = 0x0010; /**< Window comparator */
		static constexpr uint16_t REG_CONFIG_CPOL_MASK = 0x0008;		/**< CPol Mask */
		static constexpr uint16_t REG_CONFIG_CPOL_ACTVLOW = 0x0000; /**< ALERT/RDY pin is low when active (default) */
		static constexpr uint16_t REG_CONFIG_CPOL_ACTVHI = 0x0008;	/**< ALERT/RDY pin is high when active */

		static constexpr uint16_t REG_CONFIG_CLAT_MASK = 0x0004;	 /**< Determines if ALERT/RDY pin latches once asserted */
		static constexpr uint16_t REG_CONFIG_CLAT_NONLAT = 0x0000; /**< Non-latching comparator (default) */
		static constexpr uint16_t REG_CONFIG_CLAT_LATCH = 0x0004;	 /**< Latching comparator */

		static constexpr uint16_t REG_CONFIG_CQUE_MASK = 0x0003;	/**< CQue Mask */
		static constexpr uint16_t REG_CONFIG_CQUE_1CONV = 0x0000; /**< Assert ALERT/RDY after one conversion */
		static constexpr uint16_t REG_CONFIG_CQUE_2CONV = 0x0001; /**< Assert ALERT/RDY after two conversions */
		static constexpr uint16_t REG_CONFIG_CQUE_4CONV = 0x0002; /**< Assert ALERT/RDY after four conversions */
		static constexpr uint16_t REG_CONFIG_CQUE_NONE = 0x0003;	/**< Disable the comparator and put ALERT/RDY in high state (default) */

		// Data Rates for ADS1015
		static constexpr uint16_t RATE_ADS1015_128SPS = 0x0000;	 /**< 128 samples per second */
		static constexpr uint16_t RATE_ADS1015_250SPS = 0x0020;	 /**< 250 samples per second */
		static constexpr uint16_t RATE_ADS1015_490SPS = 0x0040;	 /**< 490 samples per second */
		static constexpr uint16_t RATE_ADS1015_920SPS = 0x0060;	 /**< 920 samples per second */
		static constexpr uint16_t RATE_ADS1015_1600SPS = 0x0080; /**< 1600 samples per second (default) */
		static constexpr uint16_t RATE_ADS1015_2400SPS = 0x00A0; /**< 2400 samples per second */
		static constexpr uint16_t RATE_ADS1015_3300SPS = 0x00C0; /**< 3300 samples per second */

		// Data Rates for ADS1115
		static constexpr uint16_t RATE_ADS1115_8SPS = 0x0000;		/**< 8 samples per second */
		static constexpr uint16_t RATE_ADS1115_16SPS = 0x0020;	/**< 16 samples per second */
		static constexpr uint16_t RATE_ADS1115_32SPS = 0x0040;	/**< 32 samples per second */
		static constexpr uint16_t RATE_ADS1115_64SPS = 0x0060;	/**< 64 samples per second */
		static constexpr uint16_t RATE_ADS1115_128SPS = 0x0080; /**< 128 samples per second (default) */
		static constexpr uint16_t RATE_ADS1115_250SPS = 0x00A0; /**< 250 samples per second */
		static constexpr uint16_t RATE_ADS1115_475SPS = 0x00C0; /**< 475 samples per second */
		static constexpr uint16_t RATE_ADS1115_860SPS = 0x00E0; /**< 860 samples per second */
	};

public:
	// === enums ===
	/*! 8-bit I2C address*/
	enum ADSXAddressI2C_e : uint8_t
	{
		ADSX_ADDRESS_GND = 0x48, /**< default ADDR connected to gnd*/
		ADSX_ADDRESS_VDD = 0x49, /**< ADDR connected to VDD*/
		ADSX_ADDRESS_SDA = 0x4A, /**< ADDR connected to SDA*/
		ADSX_ADDRESS_SCLK = 0x4B /**< ADDR connected to SCLK*/
	};

	/*! ADC Channel Numbers */
	enum ADSX_AINX_e : uint8_t
	{
		ADSX_AIN0 = 0, /**< Channel AIN0*/
		ADSX_AIN1 = 1, /**< Channel AIN1*/
		ADSX_AIN2 = 2, /**< Channel AIN2*/
		ADSX_AIN3 = 3	/**< Channel AIN3 */
	};

	/*! Gain Settings */
	enum ADSXGain_e : uint16_t
	{
		ADSXGain_TWOTHIRDS = ADSX::REG_CONFIG_PGA_6_144V, 
		ADSXGain_ONE = ADSX::REG_CONFIG_PGA_4_096V,
		ADSXGain_TWO = ADSX::REG_CONFIG_PGA_2_048V,
		ADSXGain_FOUR =ADSX::REG_CONFIG_PGA_1_024V,
		ADSXGain_EIGHT = ADSX::REG_CONFIG_PGA_0_512V,
		ADSXGain_SIXTEEN = ADSX::REG_CONFIG_PGA_0_256V
	};

	/*! Register Config */
	enum ADSXRegConfig_e : uint16_t
	{
		ADSXRegConfigMuxDiff_0_1 = 0x0000, // Differential P = AIN0, N = AIN1 (default)
		ADSXRegConfigMuxDiff_0_3 = 0x1000, // Differential P = AIN0, N = AIN3
		ADSXRegConfigMuxDiff_1_3 = 0x2000, // Differential P = AIN1, N = AIN3
		ADSXRegConfigMuxDiff_2_3 = 0x3000, // Differential P = AIN2, N = AIN3
		ADSXRegConfigMuxSingle_0 = 0x4000, // Single-ended AIN0
		ADSXRegConfigMuxSingle_1 = 0x5000, // Single-ended AIN1
		ADSXRegConfigMuxSingle_2 = 0x6000, // Single-ended AIN2
		ADSXRegConfigMuxSingle_3 = 0x7000	 // Single-ended AIN3
	};

	/*! Configuration mode */
	enum ADSXConfigMode_e
	{
		ADSContinuousMode = true,
		ADSSingleShotMode = false,
	};

	// === Functions ===
	bool beginADSX(ADSXAddressI2C_e addr, i2c_inst_t *type, uint16_t speed, uint8_t SDA, uint8_t SCLK, uint32_t I2CDelay);
	void deinitI2C();
	void setGain(ADSXGain_e gain);
	ADSXGain_e getGain();
	void setDataRate(uint16_t rate);
	uint16_t getDataRate();

	int16_t readADC_SingleEnded(ADSX_AINX_e channel);
	void startComparator_SingleEnded(ADSX_AINX_e channel, int16_t threshold);

	int16_t readADC_Diff01();
	int16_t readADC_Diff03();
	int16_t readADC_Diff13();
	int16_t readADC_Diff23();

	int16_t getLastConversionResults();
	void startADCReading(ADSXRegConfig_e mux, ADSXConfigMode_e mode);
	float computeVolts(int16_t counts);
	bool conversionComplete();

protected :
	uint8_t _BitShift;	 /**< bit shift amount */
	ADSXGain_e _ADCGain; /**< PGA ADC gain */
	uint16_t _DataRate;	 /**< Data rate */

private:
	i2c_inst_t *_i2c; /**< i2C port number i2c0 or i2c1 */
	uint8_t _SDataPin; /**< GPIO for I2C data pin */
	uint8_t _SClkPin;  /**< GPIO for I2C Clock pin */
	uint16_t _CLKSpeed = 100; /**< I2C bus speed in khz */
	ADSXAddressI2C_e _AddresI2C = ADSX_ADDRESS_GND;
	uint8_t _dataBuffer[3];  /**< i2C comms Data buffer  */
	uint32_t _ADSX_I2C_DELAY = 50000; /**< uS delay , I2C timeout */
	// I2C interface
	void writeRegister(uint8_t reg, uint16_t value);
	uint16_t readRegister(uint8_t reg);
};

/*! @brief ADS1015 sub class */
class PICO_ADS1015 : public PICO_ADS1X15
{
public:
	PICO_ADS1015();
};

/*! @brief ADS1115 sub class */
class PICO_ADS1115 : public PICO_ADS1X15
{
public:
	PICO_ADS1115();
};

#endif
