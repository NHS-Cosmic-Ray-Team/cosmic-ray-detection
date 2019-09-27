#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
/*=========================================================================
    I2C ADDRESS/BITS
  -----------------------------------------------------------------------*/
#define BME280_ADDRESS                       0x77
#define BME280_REGISTER_DIG_T1               0x88
#define BME280_REGISTER_DIG_T2               0x8A
#define BME280_REGISTER_DIG_T3               0x8C
#define BME280_REGISTER_DIG_P1               0x8E
#define BME280_REGISTER_DIG_P2               0x90
#define BME280_REGISTER_DIG_P3               0x92
#define BME280_REGISTER_DIG_P4               0x94
#define BME280_REGISTER_DIG_P5               0x96
#define BME280_REGISTER_DIG_P6               0x98
#define BME280_REGISTER_DIG_P7               0x9A
#define BME280_REGISTER_DIG_P8               0x9C
#define BME280_REGISTER_DIG_P9               0x9E
#define BME280_REGISTER_DIG_H1               0xA1
#define BME280_REGISTER_DIG_H2               0xE1
#define BME280_REGISTER_DIG_H3               0xE3
#define BME280_REGISTER_DIG_H4               0xE4
#define BME280_REGISTER_DIG_H5               0xE5
#define BME280_REGISTER_DIG_H6               0xE7
#define BME280_REGISTER_CHIPID               0xD0
#define BME280_REGISTER_SOFTRESET            0xE0

#define BME280_REGISTER_CONTROLHUMID         0xF2
#define BME280_REGISTER_STATUS               0XF3
#define BME280_REGISTER_CONTROL              0xF4
#define BME280_REGISTER_CONFIG               0xF5
#define BME280_REGISTER_PRESSUREDATA         0xF7
#define BME280_REGISTER_TEMPDATA             0xFA
#define BME280_REGISTER_HUMIDDATA            0xFD

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
} bme280_calib_data;

class myBME280 {
  public:
    enum sensor_sampling {
      SAMPLING_X16  = 0b101
    };

    enum sensor_mode {
      MODE_NORMAL = 0b11
    };

    enum sensor_filter {
      FILTER_OFF = 0b000,
    };

    // standby durations in ms
    enum standby_duration {
      STANDBY_MS_0_5  = 0b000,
    };

    // constructor
    myBME280() {}

    //Initialise sensor with given parameters / settings
    bool begin(uint8_t addr = BME280_ADDRESS) {

      _i2caddr = addr;
      Wire.begin();

      // check if sensor, i.e. the chip ID is correct
      if (read8(BME280_REGISTER_CHIPID) != 0x60)
        return false;

      // reset the device using soft-reset
      // this makes sure the IIR is off, etc.
      write8(BME280_REGISTER_SOFTRESET, 0xB6);

      // wait for chip to wake up.
      delay(300);

      // if chip is still reading calibration, delay
      while (isReadingCalibration())
        delay(100);

      readCoefficients(); // read trimming parameters, see DS 4.2.2

      setSampling(); // use defaults

      return true;
    }

    void setSampling(sensor_mode mode  = MODE_NORMAL,
                     sensor_sampling tempSampling  = SAMPLING_X16,
                     sensor_sampling pressSampling = SAMPLING_X16,
                     sensor_sampling humSampling   = SAMPLING_X16,
                     sensor_filter filter          = FILTER_OFF,
                     standby_duration duration     = STANDBY_MS_0_5
                    ) {
      _measReg.mode     = mode;
      _measReg.osrs_t   = tempSampling;
      _measReg.osrs_p   = pressSampling;

      _humReg.osrs_h    = humSampling;
      _configReg.filter = filter;
      _configReg.t_sb   = duration;

      // you must make sure to also set REGISTER_CONTROL after setting the
      // CONTROLHUMID register, otherwise the values won't be applied (see DS 5.4.3)
      write8(BME280_REGISTER_CONTROLHUMID, _humReg.get());
      write8(BME280_REGISTER_CONFIG, _configReg.get());
      write8(BME280_REGISTER_CONTROL, _measReg.get());
    }

    //Returns the temperature from the sensor
    float readTemperature(void)
    {
      int32_t var1, var2;

      int32_t adc_T = read24(BME280_REGISTER_TEMPDATA);
      if (adc_T == 0x800000) // value in case temp measurement was disabled
        return NAN;

      adc_T >>= 4;

      var1 = ((((adc_T >> 3) - ((int32_t)_bme280_calib.dig_T1 << 1))) *
              ((int32_t)_bme280_calib.dig_T2)) >> 11;

      var2 = (((((adc_T >> 4) - ((int32_t)_bme280_calib.dig_T1)) *
                ((adc_T >> 4) - ((int32_t)_bme280_calib.dig_T1))) >> 12) *
              ((int32_t)_bme280_calib.dig_T3)) >> 14;

      t_fine = var1 + var2;
      float T = (t_fine * 5 + 128) >> 8;
      return T / 100;
    }

    //Returns the pressure from the sensor
    float readPressure(void) {
      int64_t var1, var2, p;
      readTemperature(); // must be done first to get t_fine

      int32_t adc_P = read24(BME280_REGISTER_PRESSUREDATA);
      if (adc_P == 0x800000) // value in case pressure measurement was disabled
        return NAN;
      adc_P >>= 4;

      var1 = ((int64_t)t_fine) - 128000;
      var2 = var1 * var1 * (int64_t)_bme280_calib.dig_P6;
      var2 = var2 + ((var1 * (int64_t)_bme280_calib.dig_P5) << 17);
      var2 = var2 + (((int64_t)_bme280_calib.dig_P4) << 35);
      var1 = ((var1 * var1 * (int64_t)_bme280_calib.dig_P3) >> 8) +
             ((var1 * (int64_t)_bme280_calib.dig_P2) << 12);
      var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)_bme280_calib.dig_P1) >> 33;

      if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
      }
      p = 1048576 - adc_P;
      p = (((p << 31) - var2) * 3125) / var1;
      var1 = (((int64_t)_bme280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
      var2 = (((int64_t)_bme280_calib.dig_P8) * p) >> 19;

      p = ((p + var1 + var2) >> 8) + (((int64_t)_bme280_calib.dig_P7) << 4);
      return (float)p / 256;
    }

    //Returns the humidity from the sensor
    float readHumidity(void) {
      readTemperature(); // must be done first to get t_fine

      int32_t adc_H = read16(BME280_REGISTER_HUMIDDATA);
      if (adc_H == 0x8000) // value in case humidity measurement was disabled
        return NAN;

      int32_t v_x1_u32r;

      v_x1_u32r = (t_fine - ((int32_t)76800));

      v_x1_u32r = (((((adc_H << 14) - (((int32_t)_bme280_calib.dig_H4) << 20) -
                      (((int32_t)_bme280_calib.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                   (((((((v_x1_u32r * ((int32_t)_bme280_calib.dig_H6)) >> 10) *
                        (((v_x1_u32r * ((int32_t)_bme280_calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                      ((int32_t)2097152)) * ((int32_t)_bme280_calib.dig_H2) + 8192) >> 14));

      v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                                 ((int32_t)_bme280_calib.dig_H1)) >> 4));

      v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
      v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
      float h = (v_x1_u32r >> 12);
      return  h / 1024.0;
    }

  private:
    uint8_t   _i2caddr;
    int32_t   _sensorID;
    int32_t   t_fine;

    bme280_calib_data _bme280_calib;

    // The config register
    struct config {
      unsigned int t_sb : 3;
      unsigned int filter : 3;

      // unused - don't set
      unsigned int none : 1;

      unsigned int get() {
        return (t_sb << 5) | (filter << 3);
      }
    };
    config _configReg;

    // The ctrl_meas register
    struct ctrl_meas {
      unsigned int osrs_t : 3;

      // pressure oversampling
      unsigned int osrs_p : 3;

      // device mode
      unsigned int mode : 2;

      unsigned int get() {
        return (osrs_t << 5) | (osrs_p << 3) | mode;
      }
    };
    ctrl_meas _measReg;

    // The ctrl_hum register
    struct ctrl_hum {
      // unused - don't set
      unsigned int none : 5;

      // pressure oversampling
      unsigned int osrs_h : 3;
      unsigned int get() {
        return (osrs_h);
      }
    };
    ctrl_hum _humReg;

    //Reads the factory-set coefficients
    void readCoefficients(void)
    {
      _bme280_calib.dig_T1 = read16_LE(BME280_REGISTER_DIG_T1);
      _bme280_calib.dig_T2 = readS16_LE(BME280_REGISTER_DIG_T2);
      _bme280_calib.dig_T3 = readS16_LE(BME280_REGISTER_DIG_T3);

      _bme280_calib.dig_P1 = read16_LE(BME280_REGISTER_DIG_P1);
      _bme280_calib.dig_P2 = readS16_LE(BME280_REGISTER_DIG_P2);
      _bme280_calib.dig_P3 = readS16_LE(BME280_REGISTER_DIG_P3);
      _bme280_calib.dig_P4 = readS16_LE(BME280_REGISTER_DIG_P4);
      _bme280_calib.dig_P5 = readS16_LE(BME280_REGISTER_DIG_P5);
      _bme280_calib.dig_P6 = readS16_LE(BME280_REGISTER_DIG_P6);
      _bme280_calib.dig_P7 = readS16_LE(BME280_REGISTER_DIG_P7);
      _bme280_calib.dig_P8 = readS16_LE(BME280_REGISTER_DIG_P8);
      _bme280_calib.dig_P9 = readS16_LE(BME280_REGISTER_DIG_P9);

      _bme280_calib.dig_H1 = read8(BME280_REGISTER_DIG_H1);
      _bme280_calib.dig_H2 = readS16_LE(BME280_REGISTER_DIG_H2);
      _bme280_calib.dig_H3 = read8(BME280_REGISTER_DIG_H3);
      _bme280_calib.dig_H4 = (read8(BME280_REGISTER_DIG_H4) << 4) | (read8(BME280_REGISTER_DIG_H4 + 1) & 0xF);
      _bme280_calib.dig_H5 = (read8(BME280_REGISTER_DIG_H5 + 1) << 4) | (read8(BME280_REGISTER_DIG_H5) >> 4);
      _bme280_calib.dig_H6 = (int8_t)read8(BME280_REGISTER_DIG_H6);
    }

    //return true if chip is busy reading cal data
    bool isReadingCalibration(void)
    {
      uint8_t const rStatus = read8(BME280_REGISTER_STATUS);
      return (rStatus & (1 << 0)) != 0;
    }


    //Writes an 8 bit value over I2C
    void write8(byte reg, byte value) {
      Wire.beginTransmission((uint8_t)_i2caddr);
      Wire.write((uint8_t)reg);
      Wire.write((uint8_t)value);
      Wire.endTransmission();
    }
    //Reads an 8 bit value over I2C
    uint8_t read8(byte reg) {
      uint8_t value;
      Wire.beginTransmission((uint8_t)_i2caddr);
      Wire.write((uint8_t)reg);
      Wire.endTransmission();
      Wire.requestFrom((uint8_t)_i2caddr, (byte)1);
      value = Wire.read();
      return value;

    }
    //Reads a 16 bit value over I2C or SPI
    uint16_t read16(byte reg)
    {
      uint16_t value;
      Wire.beginTransmission((uint8_t)_i2caddr);
      Wire.write((uint8_t)reg);
      Wire.endTransmission();
      Wire.requestFrom((uint8_t)_i2caddr, (byte)2);
      value = (Wire.read() << 8) | Wire.read();
      return value;

    }
    //Reads a 24 bit value over I2C
    uint32_t read24(byte reg)
    {
      uint32_t value;
      Wire.beginTransmission((uint8_t)_i2caddr);
      Wire.write((uint8_t)reg);
      Wire.endTransmission();
      Wire.requestFrom((uint8_t)_i2caddr, (byte)3);

      value = Wire.read();
      value <<= 8;
      value |= Wire.read();
      value <<= 8;
      value |= Wire.read();
      return value;
    }
    uint16_t read16_LE(byte reg) {
      uint16_t temp = read16(reg);
      return (temp >> 8) | (temp << 8);
    }

    int16_t readS16_LE(byte reg) {
      return (int16_t)read16_LE(reg);
    }
};
//*************************End of myBME280 Class****************************************************************
//**************************************************************************************************************

#define GPS_RMC       "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"
#define GPS_RMCGGA    "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"
#define GPS_1Hz       "$PMTK220,1000*1F\r\n"
#define GPS_5Hz       "$PMTK220,200*2C\r\n"
#define GPS_10Hz      "$PMTK220,100*2F\r\n"
#define GPS_9600      "$PMTK251,9600*17\r\n"
#define GPS_14400     "$PMTK251,14400*29\r\n"
#define GPS_38400     "$PMTK251,38400*27\r\n"
#define _GPRMC_TERM   "GPRMC"
#define _GPGGA_TERM   "GPGGA"
#define COMBINE(sentence_type, term_number) (((unsigned)(sentence_type) << 5) | term_number)

//***************************** TinyGPS Class Definition ********************************************************
/*
   This is a modified TinyGPS Class of the TinyGPS Library
   TinyGPS - a small customized GPS library for Arduino providing basic NMEA parsing
   (Date, time, location[latitude and longitude], number of satellites, fix quality and latency)
   for the purpuse of our project
*/

class myGPS
{
  public:
    enum {
      GPS_INVALID_AGE = 0xFFFFFFFF,      GPS_INVALID_ANGLE = 999999999,
      GPS_INVALID_DATE = 0,              GPS_INVALID_TIME = 0xFFFFFFFF,
      GPS_INVALID_FIX_TIME = 0xFFFFFFFF, GPS_INVALID_SATELLITES = 0XFF
    };
    myGPS() //Constructor
      :  _time(GPS_INVALID_TIME)
      ,  _date(GPS_INVALID_DATE)
      ,  _latitude(GPS_INVALID_ANGLE)
      ,  _longitude(GPS_INVALID_ANGLE)
      ,  _last_time_fix(GPS_INVALID_FIX_TIME)
      ,  _last_position_fix(GPS_INVALID_FIX_TIME)
      ,  _parity(0)
      ,  _is_checksum_term(false)
      ,  _sentence_type(_GPS_SENTENCE_OTHER)
      ,  _term_number(0)
      ,  _term_offset(0)
      ,  _gps_data_good(false)
      ,  _fixquality(0)
      ,  _numsats(0)
    {
      _term[0] = '\0';
    }

    //TinyGPS &operator << (char c) {encode(c); return *this;}

    unsigned char satsinview() {
      return _numsats;  // number of satellites
    }
    unsigned char fix_Quality() {
      return _fixquality;  // fix quality
    }

    bool encode(char c)
    {
      bool valid_sentence = false;

      switch (c)
      {
        case ',': // term terminators
          _parity ^= c;
        case '\r':
        case '\n':
        case '*':
          if (_term_offset < sizeof(_term))
          {
            _term[_term_offset] = 0;
            valid_sentence = term_complete();
          }
          ++_term_number;
          _term_offset = 0;
          _is_checksum_term = c == '*';
          return valid_sentence;

        case '$': // sentence begin
          _term_number = _term_offset = 0;
          _parity = 0;
          _sentence_type = _GPS_SENTENCE_OTHER;
          _is_checksum_term = false;
          _gps_data_good = false;
          return valid_sentence;
      }

      // ordinary characters
      if (_term_offset < sizeof(_term) - 1)
        _term[_term_offset++] = c;
      if (!_is_checksum_term)
        _parity ^= c;

      return valid_sentence;
    }
    void get_position(long *latitude, long *longitude, unsigned long *fix_age)
    {
      if (latitude) *latitude = _latitude;
      if (longitude) *longitude = _longitude;
      if (fix_age) *fix_age = _last_position_fix == GPS_INVALID_FIX_TIME ?
                                GPS_INVALID_AGE : millis() - _last_position_fix;
    }

    void get_datetime(unsigned long *date, unsigned long *time, unsigned long *age)
    {
      if (date) *date = _date;
      if (time) *time = _time;
      if (age) *age = _last_time_fix == GPS_INVALID_FIX_TIME ?
                        GPS_INVALID_AGE : millis() - _last_time_fix;
    }

  private:
    enum {_GPS_SENTENCE_GPRMC, _GPS_SENTENCE_GPGGA, _GPS_SENTENCE_OTHER};

    // properties
    unsigned long _time, _new_time;
    unsigned long _date, _new_date;
    long _latitude, _new_latitude;
    long _longitude, _new_longitude;
    unsigned long _last_time_fix, _new_time_fix;
    unsigned long _last_position_fix, _new_position_fix;
    unsigned short _numsats, _new_numsats;
    unsigned short _fixquality, _new_fixquality;

    // parsing state variables
    byte _parity;
    bool _is_checksum_term;
    char _term[15];
    byte _sentence_type;
    byte _term_number;
    byte _term_offset;
    bool _gps_data_good;

    int from_hex(char a)
    {
      if (a >= 'A' && a <= 'F')
        return a - 'A' + 10;
      else if (a >= 'a' && a <= 'f')
        return a - 'a' + 10;
      else
        return a - '0';
    }

    unsigned long parse_decimal()
    {
      char *p = _term;
      bool isneg = *p == '-';
      if (isneg) ++p;
      unsigned long ret = 100UL * gpsatol(p);
      while (gpsisdigit(*p)) ++p;
      if (*p == '.')
      {
        if (gpsisdigit(p[1]))
        {
          ret += 10 * (p[1] - '0');
          if (gpsisdigit(p[2]))
            ret += p[2] - '0';
        }
      }
      return isneg ? -ret : ret;
    }

    // Parse a string in the form ddmm.mmmmmmm...
    unsigned long parse_degrees()
    {
      char *p;
      unsigned long left_of_decimal = gpsatol(_term);
      unsigned long hundred1000ths_of_minute = (left_of_decimal % 100UL) * 100000UL;
      for (p = _term; gpsisdigit(*p); ++p);
      if (*p == '.')
      {
        unsigned long mult = 10000;
        while (gpsisdigit(*++p))
        {
          hundred1000ths_of_minute += mult * (*p - '0');
          mult /= 10;
        }
      }
      return (left_of_decimal / 100) * 1000000 + (hundred1000ths_of_minute + 3) / 6;
    }

    bool term_complete()
    {
      if (_is_checksum_term)
      {
        byte checksum = 16 * from_hex(_term[0]) + from_hex(_term[1]);
        if (checksum == _parity)
        {
          if (_gps_data_good)
          {
            _last_time_fix = _new_time_fix;
            _last_position_fix = _new_position_fix;

            switch (_sentence_type)
            {
              case _GPS_SENTENCE_GPRMC:
                _time      = _new_time;
                _date      = _new_date;
                _latitude  = _new_latitude;
                _longitude = _new_longitude;
                break;

              case _GPS_SENTENCE_GPGGA:
                _numsats = _new_numsats;
                _fixquality = _new_fixquality;

                break;
            }

            return true;
          }
        }
        return false;
      }

      // the first term determines the sentence type
      if (_term_number == 0)
      {
        if (!gpsstrcmp(_term, _GPRMC_TERM))
          _sentence_type = _GPS_SENTENCE_GPRMC;
        else if (!gpsstrcmp(_term, _GPGGA_TERM))
          _sentence_type = _GPS_SENTENCE_GPGGA;
        else
          _sentence_type = _GPS_SENTENCE_OTHER;
        return false;
      }

      if (_sentence_type != _GPS_SENTENCE_OTHER && _term[0])
        switch (COMBINE(_sentence_type, _term_number))
        {
          case COMBINE(_GPS_SENTENCE_GPRMC, 1): // Time in both sentences
          case COMBINE(_GPS_SENTENCE_GPGGA, 1):
            _new_time = parse_decimal();
            _new_time_fix = millis();
            break;
          case COMBINE(_GPS_SENTENCE_GPRMC, 2): // GPRMC validity
            _gps_data_good = _term[0] == 'A';
            break;
          case COMBINE(_GPS_SENTENCE_GPRMC, 3): // Latitude
          case COMBINE(_GPS_SENTENCE_GPGGA, 2):
            _new_latitude = parse_degrees();
            _new_position_fix = millis();
            break;
          case COMBINE(_GPS_SENTENCE_GPRMC, 4): // N/S
          case COMBINE(_GPS_SENTENCE_GPGGA, 3):
            if (_term[0] == 'S')
              _new_latitude = -_new_latitude;
            break;
          case COMBINE(_GPS_SENTENCE_GPRMC, 5): // Longitude
          case COMBINE(_GPS_SENTENCE_GPGGA, 4):
            _new_longitude = parse_degrees();
            break;
          case COMBINE(_GPS_SENTENCE_GPRMC, 6): // E/W
          case COMBINE(_GPS_SENTENCE_GPGGA, 5):
            if (_term[0] == 'W')
              _new_longitude = -_new_longitude;
            break;
          case COMBINE(_GPS_SENTENCE_GPRMC, 9): // Date (GPRMC)
            _new_date = gpsatol(_term);
            break;
          case COMBINE(_GPS_SENTENCE_GPGGA, 6): // Fix data (GPGGA)
            _gps_data_good = _term[0] > '0';
            _new_fixquality = (unsigned char)atoi(_term);

            break;
          case COMBINE(_GPS_SENTENCE_GPGGA, 7): // Satellites used (GPGGA)
            _new_numsats = (unsigned char)atoi(_term);
            break;
            //case COMBINE(_GPS_SENTENCE_GPGGA, 8): // HDOP
            // _new_hdop = parse_decimal();
            //break;
            //case COMBINE(_GPS_SENTENCE_GPGGA, 9): // Altitude (GPGGA)
            //_new_altitude = parse_decimal();
            //break;
        }
      return false;
    }

    long gpsatol(const char *str)
    {
      long ret = 0;
      while (gpsisdigit(*str))
        ret = 10 * ret + *str++ - '0';
      return ret;
    }

    int gpsstrcmp(const char *str1, const char *str2)
    {
      while (*str1 && *str1 == *str2)
        ++str1, ++str2;
      return *str1;
    }
    bool gpsisdigit(char c) {
      return c >= '0' && c <= '9';
    }

};
//*************************End of TinyGPS Class***********************
myBME280 bme; // I2C
myGPS gps;
SoftwareSerial ss(8, 7);

unsigned long gpsDate, gpsTime, gpsAge;
float flat, flon;
long int  lat, lon;
unsigned long signal_1pps;
unsigned long latency;

bool newData = false;
int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0, oldSecond = -1, flag = 0, oldMinute = -1;
signed long count = 0;
int lock_minute = -1;

//---------------------------------------Subroutines-------------------------------------
void readGPS()
{
  while (ss.available()) {
    char c = ss.read();
    if (gps.encode(c)) {
      newData = true;
    }
    if (newData)
    {
      latency = micros() - signal_1pps;
      if (flag == 0)
      {
        delay(50);
        flag = 1;
      }
    }
  }
}

void initCounter()  /* initialize the counter */
{
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);
  SPI.begin();
  digitalWrite(9, LOW); // bring ss low to select and begin SPI conversation
  SPI.transfer(0x00); // write serial data to the MDR0 register
  SPI.transfer(0x00); // write serial data to the MDR1 register
  digitalWrite(9, HIGH);
  digitalWrite(9, LOW);
  SPI.transfer(0x20); // clears the CNTR
  digitalWrite(9, HIGH); // bring high to end conversation
}

long readCounter() /*Reads the Encoders to retrieve the updated value and RETURNS: long */
{
  unsigned int count_1, count_2, count_3, count_4;
  long count_value = 0;
  digitalWrite(9, LOW); // Begin SPI conversation
  SPI.transfer(0x60); // Request count
  count_1 = SPI.transfer(0x00); // Read highest order byte
  count_2 = SPI.transfer(0x00);
  count_3 = SPI.transfer(0x00);
  count_4 = SPI.transfer(0x00); // Read lowest order byte

  digitalWrite(9, HIGH); // Terminate SPI conversation
  count_value = (count_1 << 8) + count_2;
  count_value = (count_value << 8) + count_3;
  count_value = (count_value << 8) + count_4;
  return count_value;
}

void barometer()  /*subroutine for barometer sensor */
{
  Serial.print(bme.readTemperature());
  Serial.print(",");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.print(",");
  Serial.print(bme.readHumidity());
}

void outputData()
{
  year = gpsDate % 100 + 2000;
  month = (gpsDate / 100) % 100;
  day = gpsDate / 10000;
  hour = gpsTime / 1000000;
  minute = (gpsTime / 10000) % 100;
  second = (gpsTime / 100) % 100;

  Serial.print(year);
  if (month < 10)
  {
    Serial.print("0");Serial.print(month);
  }
  else
  {
    Serial.print(month);
  }
  if (day < 10)
  {
    Serial.print("0");Serial.print(day);
  }
  else
  {
    Serial.print(day);
  }
  Serial.print(",");
  if (hour < 10)
  {
    Serial.print("0");Serial.print(hour);
  }
  else
  {
    Serial.print(hour);
  }
  if (minute < 10)
  {
    Serial.print("0");Serial.print(minute);
  }
  else
  {
    Serial.print(minute);
  }
  if (second < 10)
  {
    Serial.print("0");Serial.print(second);
  }
  else
  {
    Serial.print(second);
  }
  Serial.print(",");

  count = readCounter(); //calls subroutine to reac the counter
  Serial.print(count);
  Serial.print(",");

  digitalWrite(9, LOW); //begin SPI conversation
  SPI.transfer(0x20); //clears the CNTR
  digitalWrite(9, HIGH); //end SPI conversation

  barometer();

  Serial.print(",");
  Serial.print((lat / 1000000.0));// print latitude
  Serial.print(",");
  Serial.print((lon / 1000000.0));
  Serial.print(",");
  Serial.print(gps.satsinview());
  Serial.print(",");
  Serial.print(gps.fix_Quality());
  Serial.print(",");
  Serial.println((latency / 1000.0));

  oldSecond = second;
}

void one_pps()
{
  signal_1pps = micros();
  flag = 0;
}

void setup()
{
  Serial.begin(9600);
  delay(5000);
  initCounter(); //calls the subroutine to initialize the counter
  //Need to write to the soft serial at the rate it is listening, default is 9600
  ss.begin(9600);
  ss.print(GPS_RMC);
  ss.print(GPS_RMCGGA);
  ss.print(GPS_5Hz);

  ss.print(GPS_9600);//Change the baud rate to the the new rate so you can continue the conversation
  if (!bme.begin())
  {
    //Serial.println("No BME280 sensor, check wiring!");
    while (1);
  }
  pinMode(10, OUTPUT);
  Serial.println("Date,Time,Count,TempC,Pressure,Hmdty,Lat,Lon,Numsats,FixQ,Latency");
  attachInterrupt(0, one_pps, RISING); //1PPS signal to pin 2, rising signal.
}

void loop()
{
  readGPS();
  gps.get_datetime(&gpsDate, &gpsTime, &gpsAge);
  gps.get_position(&lat, &lon, &gpsAge);
  minute = (gpsTime / 10000) % 100;
  hour = gpsTime / 1000000;
  int index = (hour * 60) + minute;

  if (gpsAge <= 1000)
  {
    if ((index == 0) && (minute != lock_minute)) //check that it's midnight
    {
      lock_minute = minute;
      Serial.println("Date,Time,Count,TempC,Pressure,Hmdty,Lat,Lon,Numsats,FixQ,Latency");
      outputData();
    }

    if ((index % 1 == 0) && (minute != lock_minute))//check that minute is 5
    {
      lock_minute = minute;
      outputData();
    }
  }

}

