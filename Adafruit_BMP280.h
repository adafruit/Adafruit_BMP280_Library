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
#ifndef __BMP280_H__
#define __BMP280_H__

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Adafruit_Sensor.h>

#ifdef __AVR_ATtiny85__
 #include "TinyWireM.h"
 #define Wire TinyWireM
#else
 #include <Wire.h>
#endif

/*=========================================================================
    I2C ADDRESS/BITS/SETTINGS
    -----------------------------------------------------------------------*/
    #define BMP280_ADDRESS                (0x77)
    #define BMP280_ADDRESS1               (0x76)
    #define BMP280_CHIPID                 (0x58)
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    enum
    {
      BMP280_REGISTER_DIG_T1              = 0x88,
      BMP280_REGISTER_DIG_T2              = 0x8A,
      BMP280_REGISTER_DIG_T3              = 0x8C,

      BMP280_REGISTER_DIG_P1              = 0x8E,
      BMP280_REGISTER_DIG_P2              = 0x90,
      BMP280_REGISTER_DIG_P3              = 0x92,
      BMP280_REGISTER_DIG_P4              = 0x94,
      BMP280_REGISTER_DIG_P5              = 0x96,
      BMP280_REGISTER_DIG_P6              = 0x98,
      BMP280_REGISTER_DIG_P7              = 0x9A,
      BMP280_REGISTER_DIG_P8              = 0x9C,
      BMP280_REGISTER_DIG_P9              = 0x9E,

      BMP280_REGISTER_CHIPID             = 0xD0,
      BMP280_REGISTER_VERSION            = 0xD1,
      BMP280_REGISTER_SOFTRESET          = 0xE0,

      BMP280_REGISTER_CAL26              = 0xE1,  // R calibration stored in 0xE1-0xF0

      BMP280_REGISTER_CONTROL            = 0xF4,
      BMP280_REGISTER_CONFIG             = 0xF5,
      BMP280_REGISTER_PRESSUREDATA       = 0xF7,
      BMP280_REGISTER_TEMPDATA           = 0xFA,
    };

/*=========================================================================*/

/*=========================================================================
    CALIBRATION DATA
    -----------------------------------------------------------------------*/
    typedef struct
    {
      uint16_t dig_T1;
      int16_t  dig_T2;
      int16_t  dig_T3;

      uint16_t dig_P1;
      int16_t  dig_P2;
      int16_t  dig_P3;
      int16_t  dig_P4;
      int16_t  dig_P5;
      int16_t  dig_P6;
      int16_t  dig_P7;
      int16_t  dig_P8;
      int16_t  dig_P9;

      uint8_t  dig_H1;
      int16_t  dig_H2;
      uint8_t  dig_H3;
      int16_t  dig_H4;
      int16_t  dig_H5;
      int8_t   dig_H6;
    } bmp280_calib_data;
/*=========================================================================*/

class Adafruit_BMP280
{
  public:
  
     enum sensor_sampling {
            SAMPLING_NONE = 0x00,
            SAMPLING_X1   = 0x01,
            SAMPLING_X2   = 0x02,
            SAMPLING_X4   = 0x03,
            SAMPLING_X8   = 0x04,
            SAMPLING_X16  = 0x05
        };

        enum sensor_mode {
            MODE_SLEEP  = 0x00,
            MODE_FORCED = 0x01,
            MODE_NORMAL = 0x03,
            MODE_SOFT_RESET_CODE = 0xB6
        };

        enum sensor_filter {
            FILTER_OFF = 0x00,
            FILTER_X2  = 0x01,
            FILTER_X4  = 0x02,
            FILTER_X8  = 0x03,
            FILTER_X16 = 0x04
        };

        // standby durations in ms 
        enum standby_duration {
            STANDBY_MS_1      = 0x00,
            STANDBY_MS_63     = 0x01,
            STANDBY_MS_125    = 0x02,
            STANDBY_MS_250    = 0x03,
            STANDBY_MS_500    = 0x04,
            STANDBY_MS_1000   = 0x05,
            STANDBY_MS_2000   = 0x06,
            STANDBY_MS_4000   = 0x07
        };

    Adafruit_BMP280();
    Adafruit_BMP280(int8_t cspin);
    Adafruit_BMP280(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin);

    bool  begin(uint8_t addr = BMP280_ADDRESS, uint8_t chipid = BMP280_CHIPID);
    float readTemperature(void);
    float readPressure(void);
    float readAltitude(float seaLevelhPa = 1013.25);
    void takeForcedMeasurement();    
    void setSampling(sensor_mode mode      = MODE_NORMAL,
			 sensor_sampling tempSampling  = SAMPLING_X16,
			 sensor_sampling pressSampling = SAMPLING_X16,
			 sensor_filter filter          = FILTER_OFF,
			 standby_duration duration     = STANDBY_MS_1
			 );

  private:

    void readCoefficients(void);
    uint8_t spixfer(uint8_t x);

    void      write8(byte reg, byte value);
    uint8_t   read8(byte reg);
    uint16_t  read16(byte reg);
    uint32_t  read24(byte reg);
    int16_t   readS16(byte reg);
    uint16_t  read16_LE(byte reg); // little endian
    int16_t   readS16_LE(byte reg); // little endian

    uint8_t   _i2caddr;
    int32_t   _sensorID;
    int32_t t_fine;

    int8_t _cs, _mosi, _miso, _sck;

    bmp280_calib_data _bmp280_calib;

    // The config register
    struct config
    {
        // inactive duration (standby time) in normal mode
        unsigned int t_sb : 3;

        // filter settings
        unsigned int filter : 3;

        // unused - don't set
        unsigned int none : 1;
        unsigned int spi3w_en : 1;

        unsigned int get()
        {
            return (t_sb << 5) | (filter << 3) | spi3w_en;
        }
    };
    config _configReg;

    // The ctrl_meas register
    struct ctrl_meas
    {
        // temperature oversampling
        unsigned int osrs_t : 3;

        // pressure oversampling
        unsigned int osrs_p : 3;

        // device mode
        unsigned int mode : 2;

        unsigned int get()
        {
            return (osrs_t << 5) | (osrs_p << 3) | mode;
        }
    };
    ctrl_meas _measReg;
};

#endif
