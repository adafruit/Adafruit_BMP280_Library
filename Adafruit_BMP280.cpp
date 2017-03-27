/***************************************************************************
  This is a library for the BMP280 pressure sensor

  Designed specifically to work with the Adafruit BMP280 Breakout
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_BMP280.h"


/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/


Adafruit_BMP280::Adafruit_BMP280()
  : _cs(-1), _mosi(-1), _miso(-1), _sck(-1)
{ }

Adafruit_BMP280::Adafruit_BMP280(int8_t cspin)
  : _cs(cspin), _mosi(-1), _miso(-1), _sck(-1)
{ }

Adafruit_BMP280::Adafruit_BMP280(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin)
  : _cs(cspin), _mosi(mosipin), _miso(misopin), _sck(sckpin)
{ }


bool Adafruit_BMP280::begin(uint8_t a, uint8_t chipid) {
  _i2caddr = a;

  if (_cs == -1) {
    // i2c
    Wire.begin();
  } else {
    digitalWrite(_cs, HIGH);
    pinMode(_cs, OUTPUT);

    if (_sck == -1) {
      // hardware SPI
      SPI.begin();
    } else {
      // software SPI
      pinMode(_sck, OUTPUT);
      pinMode(_mosi, OUTPUT);
      pinMode(_miso, INPUT);
    }
  }

  if (read8(BMP280_REGISTER_CHIPID) != chipid)
    return false;

  readCoefficients();
  write8(BMP280_REGISTER_CONTROL, 0x3F);
  return true;
}

uint8_t Adafruit_BMP280::spixfer(uint8_t x) {
  if (_sck == -1)
    return SPI.transfer(x);

  // software spi
  //Serial.println("Software SPI");
  uint8_t reply = 0;
  for (int i=7; i>=0; i--) {
    reply <<= 1;
    digitalWrite(_sck, LOW);
    digitalWrite(_mosi, x & (1<<i));
    digitalWrite(_sck, HIGH);
    if (digitalRead(_miso))
      reply |= 1;
  }
  return reply;
}

/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C/SPI
*/
/**************************************************************************/
void Adafruit_BMP280::write8(byte reg, byte value)
{
  if (_cs == -1) {
    Wire.beginTransmission((uint8_t)_i2caddr);
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
    Wire.endTransmission();
  } else {
    if (_sck == -1)
      SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg & ~0x80); // write, bit 7 low
    spixfer(value);
    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      SPI.endTransaction();              // release the SPI bus
  }
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C/SPI
*/
/**************************************************************************/
uint8_t Adafruit_BMP280::read8(byte reg)
{
  uint8_t value;

  if (_cs == -1) {
    Wire.beginTransmission((uint8_t)_i2caddr);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)_i2caddr, (byte)1);
    value = Wire.read();

  } else {
    if (_sck == -1)
      SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg | 0x80); // read, bit 7 high
    value = spixfer(0);
    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      SPI.endTransaction();              // release the SPI bus
  }
  return value;
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit value over I2C/SPI
*/
/**************************************************************************/
uint16_t Adafruit_BMP280::read16(byte reg)
{
  uint16_t value;

  if (_cs == -1) {
    Wire.beginTransmission((uint8_t)_i2caddr);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)_i2caddr, (byte)2);
    value = (Wire.read() << 8) | Wire.read();

  } else {
    if (_sck == -1)
      SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg | 0x80); // read, bit 7 high
    value = (spixfer(0) << 8) | spixfer(0);
    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      SPI.endTransaction();              // release the SPI bus
  }

  return value;
}

uint16_t Adafruit_BMP280::read16_LE(byte reg) {
  uint16_t temp = read16(reg);
  return (temp >> 8) | (temp << 8);

}

/**************************************************************************/
/*!
    @brief  Reads a signed 16 bit value over I2C/SPI
*/
/**************************************************************************/
int16_t Adafruit_BMP280::readS16(byte reg)
{
  return (int16_t)read16(reg);

}

int16_t Adafruit_BMP280::readS16_LE(byte reg)
{
  return (int16_t)read16_LE(reg);

}


/**************************************************************************/
/*!
    @brief  Reads a 24 bit value over I2C/SPI
*/
/**************************************************************************/
uint32_t Adafruit_BMP280::read24(byte reg)
{
  uint32_t value;

  if (_cs == -1) {
    Wire.beginTransmission((uint8_t)_i2caddr);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)_i2caddr, (byte)3);

    value = Wire.read();
    value <<= 8;
    value |= Wire.read();
    value <<= 8;
    value |= Wire.read();

  } else {
    if (_sck == -1)
      SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg | 0x80); // read, bit 7 high

    value = spixfer(0);
    value <<= 8;
    value |= spixfer(0);
    value <<= 8;
    value |= spixfer(0);

    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      SPI.endTransaction();              // release the SPI bus
  }

  return value;
}


/**************************************************************************/
/*!
    @brief  Reads up to 3 24 bit values from the same sample in burst mode.
*/
/**************************************************************************/
Adafruit_BMP280::UncompensatedData Adafruit_BMP280::readSensors(int readType)
{
  UncompensatedData results;
  byte reg = BMP280_REGISTER_TEMPDATA; // always start with this register
  byte length = 3;
  switch (readType) {
    case READ_TP:
      length = 6;
      break;
    // case READ_T:
    // default:
    //   length = 3;
    //   break;
  }

  if (_cs == -1) {
    Wire.beginTransmission((uint8_t)_i2caddr);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)_i2caddr, length);

    //if (length > 0) {
      results.temp_ADC = read24I2C();
    //}

    if (length > 3) {
      results.press_ADC = read24I2C();
    }

  } else {
    if (_sck == -1)
      SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg | 0x80); // read, bit 7 high

    //if (length > 0) {
      results.temp_ADC = read24SPI();
    //}

    if (length > 3) {
      results.press_ADC = read24SPI();
    }

    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      SPI.endTransaction();              // release the SPI bus
  }

  return results;
}

uint32_t Adafruit_BMP280::read24I2C(void) {
  uint32_t value;

  value = Wire.read();
  value <<= 8;
  value |= Wire.read();
  value <<= 8;
  value |= Wire.read();
  return value;
}

uint32_t Adafruit_BMP280::read24SPI(void) {
  uint32_t value;

  value = spixfer(0);
  value <<= 8;
  value |= spixfer(0);
  value <<= 8;
  value |= spixfer(0);
  return value;
}

/**************************************************************************/
/*!
    @brief  Reads the factory-set coefficients
*/
/**************************************************************************/
void Adafruit_BMP280::readCoefficients(void)
{
    _bmp280_calib.dig_T1 = read16_LE(BMP280_REGISTER_DIG_T1);
    _bmp280_calib.dig_T2 = readS16_LE(BMP280_REGISTER_DIG_T2);
    _bmp280_calib.dig_T3 = readS16_LE(BMP280_REGISTER_DIG_T3);

    _bmp280_calib.dig_P1 = read16_LE(BMP280_REGISTER_DIG_P1);
    _bmp280_calib.dig_P2 = readS16_LE(BMP280_REGISTER_DIG_P2);
    _bmp280_calib.dig_P3 = readS16_LE(BMP280_REGISTER_DIG_P3);
    _bmp280_calib.dig_P4 = readS16_LE(BMP280_REGISTER_DIG_P4);
    _bmp280_calib.dig_P5 = readS16_LE(BMP280_REGISTER_DIG_P5);
    _bmp280_calib.dig_P6 = readS16_LE(BMP280_REGISTER_DIG_P6);
    _bmp280_calib.dig_P7 = readS16_LE(BMP280_REGISTER_DIG_P7);
    _bmp280_calib.dig_P8 = readS16_LE(BMP280_REGISTER_DIG_P8);
    _bmp280_calib.dig_P9 = readS16_LE(BMP280_REGISTER_DIG_P9);
}

/**************************************************************************/
/*!
    @brief  Read the sensor values, individually or as a group
*/
/**************************************************************************/
float Adafruit_BMP280::readTemperature(void) {
  UncompensatedData results = readSensors(READ_T);
  return compensateTemperature(results.temp_ADC);
}

float Adafruit_BMP280::readPressure(void) {
  UncompensatedData results = readSensors(READ_TP);
  compensateTemperature(results.temp_ADC); // must be done first to get t_fine
  return compensatePressure(results.press_ADC);
}

void Adafruit_BMP280::readAll(float *pT, float *pP) {
  UncompensatedData results = readSensors(READ_TP);
  float V = compensateTemperature(results.temp_ADC); // must be done first to get t_fine
  if (pT) *pT = V;
  if (pP) *pP = compensatePressure(results.press_ADC);
}

/**************************************************************************/
/*!
  @brief  Converts raw temperature reading using factory compensation
*/
/**************************************************************************/
float Adafruit_BMP280::compensateTemperature(int32_t adc_T)
{
  int32_t var1, var2;

  adc_T >>= 4;

  var1  = ((((adc_T>>3) - ((int32_t)_bmp280_calib.dig_T1 <<1))) *
	   ((int32_t)_bmp280_calib.dig_T2)) >> 11;

  var2  = (((((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1)) *
	     ((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1))) >> 12) *
	   ((int32_t)_bmp280_calib.dig_T3)) >> 14;

  t_fine = var1 + var2;

  float T  = (t_fine * 5 + 128) >> 8;
  return T/100;
}

/**************************************************************************/
/*!
  @brief  Converts raw pressure reading using factory compensation
*/
/**************************************************************************/
float Adafruit_BMP280::compensatePressure(int32_t adc_P) {
  int64_t var1, var2, p;

  adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_bmp280_calib.dig_P6;
  var2 = var2 + ((var1*(int64_t)_bmp280_calib.dig_P5)<<17);
  var2 = var2 + (((int64_t)_bmp280_calib.dig_P4)<<35);
  var1 = ((var1 * var1 * (int64_t)_bmp280_calib.dig_P3)>>8) +
    ((var1 * (int64_t)_bmp280_calib.dig_P2)<<12);
  var1 = (((((int64_t)1)<<47)+var1))*((int64_t)_bmp280_calib.dig_P1)>>33;

  if (var1 == 0) {
    return 0;  // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125) / var1;
  var1 = (((int64_t)_bmp280_calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((int64_t)_bmp280_calib.dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib.dig_P7)<<4);
  return (float)p/256;
}

/**************************************************************************/
/*!
    Calculates the altitude (in meters) from the specified atmospheric
    pressure (in hPa), and sea-level pressure (in hPa).

    @param  seaLevel      Sea-level pressure in hPa
    @param  atmospheric   Atmospheric pressure in hPa
*/
/**************************************************************************/
float Adafruit_BMP280::readAltitude(float seaLevel)
{
  // Equation taken from BMP180 datasheet (page 16):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude.  See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

  float atmospheric = readPressure() / 100.0F;
  return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

/**************************************************************************/
/*!
    Calculates the pressure at sea level (in hPa) from the specified altitude
    (in meters), and atmospheric pressure (in hPa).
    @param  altitude      Altitude in meters
    @param  atmospheric   Atmospheric pressure in hPa
*/
/**************************************************************************/
float Adafruit_BMP280::seaLevelForAltitude(float altitude, float atmospheric)
{
  // Equation taken from BMP180 datasheet (page 17):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude.  See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

  return atmospheric / pow(1.0 - (altitude/44330.0), 5.255);
}
