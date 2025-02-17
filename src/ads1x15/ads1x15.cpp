/*!
 * @file ads1x15.cpp
 * @brief
 * ADC ADS1015 ADS1115 library cpp file for RPI Rp2040 PICO C++ SDK
 * URL: https://github.com/gavinlyonsrepo/ADS1x15_PICO
 */

#include "../../include/ads1x15/ads1x15.hpp"

// Constructor ADS1015 sub class
PICO_ADS1015::PICO_ADS1015()
{
	_BitShift = 4;
	_ADCGain = PICO_ADS1X15::ADSXGain_TWOTHIRDS;
	_DataRate = ADSX::RATE_ADS1015_1600SPS;
}

// Constructor ADS1115 sub class
PICO_ADS1115::PICO_ADS1115()
{
	_BitShift = 0;
	_ADCGain = PICO_ADS1X15::ADSXGain_TWOTHIRDS;
	_DataRate = ADSX::RATE_ADS1115_128SPS;
}

// @brief Sets up the I2C interface
// @param i2c_addr, enum ADSX_AddressI2C_e : I2C address 8 bit address
// @param i2c_type i2c_inst_t* : I2C instance of port, IC20 or I2C1
// @param CLKspeed uint16_t : I2C Bus Clock speed in KHz. see 8.5.1.3 datasheet
// @param SDApin : I2C Data pin
// @param SCLKpin, uint8_t : I2C Clock pin
// @param I2CDelay I2C timeout in uS 
// @return : bool :true if successful, otherwise false
bool PICO_ADS1X15::beginADSX(ADSXAddressI2C_e i2c_addr, i2c_inst_t *i2c_type, uint16_t CLKspeed, uint8_t SDApin, uint8_t SCLKpin, uint32_t I2CDelay)
{
	_AddresI2C = i2c_addr;
	_i2c = i2c_type;
	_SClkPin = SCLKpin;
	_SDataPin = SDApin;
	_CLKSpeed = CLKspeed;
	_ADSX_I2C_DELAY = I2CDelay;

	int ReturnCode = 0;
	uint8_t rxData = 0;

	// init I2c pins and interface
	gpio_set_function(_SDataPin, GPIO_FUNC_I2C);
	gpio_set_function(_SClkPin, GPIO_FUNC_I2C);
	gpio_pull_up(_SDataPin);
	gpio_pull_up(_SClkPin);
	i2c_init(_i2c, _CLKSpeed * 1000);

	// check connection?
	ReturnCode = i2c_read_timeout_us(_i2c, _AddresI2C, &rxData, 1, false, _ADSX_I2C_DELAY);
	if (ReturnCode < 1)
	{ // no bytes read back from device or error issued
#ifdef ADS_SERIAL_DEBUG
		printf("1201  PICO_ADS1X15::begin: \r\n");
		printf("Check Connection, Return code :: %d ,RX data :: %u \r\n", ReturnCode, rxdata);
#endif
		return false;
	}
	return true;
}

// Switch off the  I2C
void PICO_ADS1X15::deinitI2C()
{
	gpio_set_function(_SDataPin, GPIO_FUNC_NULL);
	gpio_set_function(_SClkPin, GPIO_FUNC_NULL);
	i2c_deinit(_i2c);
}

// @brief Sets the gain and input voltage range
// @param enum ADSXGain_e , Gain setting to use
void PICO_ADS1X15::setGain(ADSXGain_e gain) { _ADCGain = gain; }

// @brief Gets the gain and input voltage range
// @return enum ADSXGain_e , Gain setting in use
PICO_ADS1X15::ADSXGain_e PICO_ADS1X15::getGain() { return _ADCGain; }

// @brief Sets the data rate
// @param rate uint16_t : Data rate to use
void PICO_ADS1X15::setDataRate(uint16_t rate) { _DataRate = rate; }

// @brief Gets the current data rate
// @return  uint16_t : Data rate in use
uint16_t PICO_ADS1X15::getDataRate() { return _DataRate; }

// @brief Gets a single-ended ADC reading from the specified channel
// @param channel enum ADSX_AINX_e : Adc channel to read 0-3
// @return ADC reading
int16_t PICO_ADS1X15::readADC_SingleEnded(ADSX_AINX_e channel)
{
	if (channel > 3)
	{
		return 0;
	}

	switch (channel)
	{
	case 0:
		startADCReading(ADSXRegConfigMuxSingle_0, ADSSingleShotMode);
		break;
	case 1:
		startADCReading(ADSXRegConfigMuxSingle_1, ADSSingleShotMode);
		break;
	case 2:
		startADCReading(ADSXRegConfigMuxSingle_2, ADSSingleShotMode);
		break;
	case 3:
		startADCReading(ADSXRegConfigMuxSingle_3, ADSSingleShotMode);
		break;
	}

	// Wait for the conversion to complete
	while (!conversionComplete())
		;
	// Read the conversion results
	return getLastConversionResults();
}

// @brief Reads the conversion results difference between the P (AIN0) and N (AIN1) input.
// @return int16_t : The ADC reading a signed value since the difference
// can be either positive or negative.
int16_t PICO_ADS1X15::readADC_Diff01()
{
	startADCReading(ADSXRegConfigMuxDiff_0_1, ADSSingleShotMode);
	while (!conversionComplete())
		; // Wait for the conversion to complete

	return getLastConversionResults(); // Read the conversion results
}

// @brief Reads the conversion results difference between the P (AIN0) and N (AIN3) input.
// @return int16_t : The ADC reading a signed value since the difference
// can be either positive or negative.
int16_t PICO_ADS1X15::readADC_Diff03()
{
	startADCReading(ADSXRegConfigMuxDiff_0_3, ADSSingleShotMode);

	// Wait for the conversion to complete
	while (!conversionComplete())
		;

	// Read the conversion results
	return getLastConversionResults();
}

// @brief Reads the conversion results difference between the P (AIN1) and N (AIN3) input.
// @return int16_t : The ADC reading a signed value since the difference
// can be either positive or negative.
int16_t PICO_ADS1X15::readADC_Diff13()
{
	startADCReading(ADSXRegConfigMuxDiff_1_3, ADSSingleShotMode);

	// Wait for the conversion to complete
	while (!conversionComplete())
		;

	// Read the conversion results
	return getLastConversionResults();
}

// @brief Reads the conversion results difference between the P (AIN2) and N (AIN3) input.
// @return :: int16_t : The ADC reading a signed value since the difference
// can be either positive or negative.
int16_t PICO_ADS1X15::readADC_Diff23()
{
	startADCReading(ADSXRegConfigMuxDiff_2_3, ADSSingleShotMode);

	// Wait for the conversion to complete
	while (!conversionComplete())
		;
	// Read the conversion results
	return getLastConversionResults();
}

// @brief Sets up the comparator to operate in basic mode
// @param channel enum ADSX_AINX_e : ADC channel to use 0-3
// @param threshold int16_t : threshold comparator
// @note  the ALERT/RDY pin to assert (go from high to low) when the ADC
// value exceeds the specified threshold.
// This will also set the ADC in continuous conversion mode.
void PICO_ADS1X15::startComparator_SingleEnded(ADSX_AINX_e channel,
																							 int16_t threshold)
{
	uint16_t configuration =
			ADSX::REG_CONFIG_CQUE_1CONV |	 // Comparator enabled and asserts on 1
																		 // match
			ADSX::REG_CONFIG_CLAT_LATCH |	 // Latching mode
			ADSX::REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
			ADSX::REG_CONFIG_CMODE_TRAD |	 // Traditional comparator (default val)
			ADSX::REG_CONFIG_MODE_CONTIN |	 // Continuous conversion mode
			ADSX::REG_CONFIG_MODE_CONTIN;	 // Continuous conversion mode

	configuration |= _ADCGain;
	configuration |= _DataRate;
	switch (channel)
	{
	case 0:
		configuration |= ADSXRegConfigMuxSingle_0;
		break;
	case 1:
		configuration |= ADSXRegConfigMuxSingle_1;
		break;
	case 2:
		configuration |= ADSXRegConfigMuxSingle_2;
		break;
	case 3:
		configuration |= ADSXRegConfigMuxSingle_3;
		break;
	}

	// Set the high threshold register
	// Shift 12-bit results left 4 bits for the ADS1015
	writeRegister(ADSX::REG_POINTER_HITHRESH, threshold << _BitShift);
	// Write config register to the ADC
	writeRegister(ADSX::REG_POINTER_CONFIG, configuration);
}

// @brief In order to clear the comparator, we need to read the conversion results.
// This function reads the last conversion results without changing the config value.
// @return :: the last ADC reading
int16_t PICO_ADS1X15::getLastConversionResults()
{
	// Read the conversion results
	uint16_t converisonResults = readRegister(ADSX::REG_POINTER_CONVERT) >> _BitShift;
	if (_BitShift == 0)
	{
		return (int16_t)converisonResults;
	}
	else
	{
		// Shift 12-bit results right 4 bits for the ADS1015,
		// making sure we keep the sign bit intact
		if (converisonResults > 0x07FF)
		{
			// negative number - extend the sign to 16th bit
			converisonResults |= 0xF000;
		}
		return (int16_t)converisonResults;
	}
}

// @brief @return true if conversion is complete, false otherwise
// @param: The ADC reading in raw counts
// @return :: the ADC reading in voltage.
// @note :: see data sheet Table 3
float PICO_ADS1X15::computeVolts(int16_t counts)
{
	float FullScaleRange;
	switch (_ADCGain)
	{
	case ADSXGain_TWOTHIRDS:
		FullScaleRange = 6.144f;
		break;
	case ADSXGain_ONE:
		FullScaleRange = 4.096f;
		break;
	case ADSXGain_TWO:
		FullScaleRange = 2.048f;
		break;
	case ADSXGain_FOUR:
		FullScaleRange = 1.024f;
		break;
	case ADSXGain_EIGHT:
		FullScaleRange = 0.512f;
		break;
	case ADSXGain_SIXTEEN:
		FullScaleRange = 0.256f;
		break;
	default:
		FullScaleRange = 0.0f;
	}
	return counts * (FullScaleRange / (32768 >> _BitShift));
}

// @brief Non-blocking start conversion function
// @param mux enum ADSXRegConfig_e :: Mux field value
// @param ConfigMode enum :: Config mode , Continuous conversion or single shot
// @note :: Call getLastConversionResults() once conversionComplete() @return true.
// In continuous mode, getLastConversionResults() will always return the
// latest result. ALERT/RDY pin is set to RDY mode, and a 8us pulse is generated every
// time new data is ready.
void PICO_ADS1X15::startADCReading(ADSXRegConfig_e mux, ADSXConfigMode_e ConfigMode)
{
	uint16_t configuration =
			ADSX::REG_CONFIG_CQUE_1CONV |	 // Set CQUE to any value other than
																		 // None so we can use it in RDY mode
			ADSX::REG_CONFIG_CLAT_NONLAT |	 // Non-latching (default val)
			ADSX::REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
			ADSX::REG_CONFIG_CMODE_TRAD;		 // Traditional comparator (default val)

	if (ConfigMode == ADSContinuousMode)
	{
		configuration |= ADSX::REG_CONFIG_MODE_CONTIN;
	}
	else
	{
		configuration |= ADSX::REG_CONFIG_MODE_SINGLE;
	}

	configuration |= _ADCGain;
	configuration |= _DataRate;
	configuration |= mux; // Set channel

	// Set start single-conversion bit
	configuration |= ADSX::REG_CONFIG_OS_SINGLE;
	// Write config register to the ADC
	writeRegister(ADSX::REG_POINTER_CONFIG, configuration);
	// Set ALERT/RDY to RDY mode.
	writeRegister(ADSX::REG_POINTER_HITHRESH, 0x8000);
	writeRegister(ADSX::REG_POINTER_LOWTHRESH, 0x0000);
}

// @brief Checks whether conversion is complete
// @return :: bool : true if conversion is complete, false otherwise
bool PICO_ADS1X15::conversionComplete()
{
	return (readRegister(ADSX::REG_POINTER_CONFIG) & 0x8000) != 0;
}

// @brief Write 16-bits to the specified destination register
// @param reg  uint8_t : register address to write
// @param value uint16_t : Value to write to register
void PICO_ADS1X15::writeRegister(uint8_t reg, uint16_t value)
{
	_dataBuffer[0] = reg;
	_dataBuffer[1] = value >> 8;
	_dataBuffer[2] = value & 0xFF;
	int ReturnCode = 0;

	ReturnCode = i2c_write_timeout_us(_i2c, _AddresI2C, _dataBuffer, 3, false, _ADSX_I2C_DELAY);
	if (ReturnCode < 1)
	{
#ifdef ADS_SERIAL_DEBUG
		printf("1203 data: \r\n");
		printf("I2C error :: writeRegister \r\n");
		printf("Tranmission code : %d \r\n", ReturnCode);
		busy_wait_ms(100);
#endif
	}
}

// @brief Read 16-bits from the specified destination register
// @param registerRead uint8_t : register address to read from
//@return uint16_t : Value to read to register
uint16_t PICO_ADS1X15::readRegister(uint8_t registerRead)
{
	_dataBuffer[0] = registerRead;

	int ReturnCode = 0;
	ReturnCode = i2c_write_timeout_us(_i2c, _AddresI2C, _dataBuffer, 1, false, _ADSX_I2C_DELAY);
	if (ReturnCode < 1)
	{
#ifdef ADS_SERIAL_DEBUG
		printf("1201 error I2C readRegister A: \r\n");
		printf("Tranmission code : %d \r\n", ReturnCode);
		busy_wait_ms(100);
#endif
	}
	ReturnCode = 0;

	ReturnCode = i2c_read_timeout_us(_i2c, _AddresI2C, _dataBuffer, 2, false, _ADSX_I2C_DELAY);
	if (ReturnCode < 1)
	{ // no bytes read back from device or error issued
#ifdef ADS_SERIAL_DEBUG
		printf("1202 I2C Error readRegister B: \r\n");
		printf("Tranmission Code :: %d\r\n", ReturnCode);
		busy_wait_ms(100);
#endif
	}

	return ((_dataBuffer[0] << 8) | _dataBuffer[1]);
}
