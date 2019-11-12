/*
    CosmicRayExtras.h

    Header file for the external classes in the cosmic ray code.

    DISCLAIMER:
    I am just repackaging existing code from TK Doe and Joe Sundermier. I take
    no credit for any of the classes they wrote, nor do I have much of an idea 
    of how they work

    Author: Kyle Mitard
    Created 11 Nov 2019
*/

#ifndef CosmicRayExtras_h
#define CosmicRayExtras_h
#endif

/*=========================================================================
    I2C ADDRESS/BITS
  -----------------------------------------------------------------------*/
#define BME280_ADDRESS						0x77
#define BME280_REGISTER_DIG_T1				0x88
#define BME280_REGISTER_DIG_T2				0x8A
#define BME280_REGISTER_DIG_T3				0x8C
#define BME280_REGISTER_DIG_P1				0x8E
#define BME280_REGISTER_DIG_P2				0x90
#define BME280_REGISTER_DIG_P3				0x92
#define BME280_REGISTER_DIG_P4				0x94
#define BME280_REGISTER_DIG_P5				0x96
#define BME280_REGISTER_DIG_P6				0x98
#define BME280_REGISTER_DIG_P7				0x9A
#define BME280_REGISTER_DIG_P8				0x9C
#define BME280_REGISTER_DIG_P9				0x9E
#define BME280_REGISTER_DIG_H1				0xA1
#define BME280_REGISTER_DIG_H2				0xE1
#define BME280_REGISTER_DIG_H3				0xE3
#define BME280_REGISTER_DIG_H4				0xE4
#define BME280_REGISTER_DIG_H5				0xE5
#define BME280_REGISTER_DIG_H6				0xE7
#define BME280_REGISTER_CHIPID				0xD0
#define BME280_REGISTER_SOFTRESET			0xE0

#define BME280_REGISTER_CONTROLHUMID		0xF2
#define BME280_REGISTER_STATUS				0XF3
#define BME280_REGISTER_CONTROL				0xF4
#define BME280_REGISTER_CONFIG				0xF5
#define BME280_REGISTER_PRESSUREDATA		0xF7
#define BME280_REGISTER_TEMPDATA			0xFA
#define BME280_REGISTER_HUMIDDATA			0xFD

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

/*=========================================================================
	myBME280 Class (based on Adafruit_BME280)
---------------------------------------------------------------------------*/

class myBME280
{
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
  
	myBME280();
	bool begin();
	void setSampling(sensor_mode mode  = MODE_NORMAL,
					sensor_sampling tempSampling  = SAMPLING_X16,
					sensor_sampling pressSampling = SAMPLING_X16,
					sensor_sampling humSampling   = SAMPLING_X16,
					sensor_filter filter          = FILTER_OFF,
					standby_duration duration     = STANDBY_MS_0_5);
	float readTemperature(void);
	float readPressure(void);
	float readHumidity(void);

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

	void readCoefficients(void);
	bool isReadingCalibration(void);
	void write8(byte reg, byte value);
	uint8_t read8(byte reg);
	uint16_t read16(byte reg);
	uint32_t read24(byte reg);
	uint16_t read16_LE(byte reg);
	int16_t readS16_LE(byte reg);
};


/*==============================================================================
	SOME STUFF FOR THE TINYGPS LIBRARY
--------------------------------------------------------------------------------*/

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

/*=========================================================================
	myGPS Class (based on TinyGPS)
---------------------------------------------------------------------------*/

class myGPS
{
public:
	myGPS();
	unsigned char satsinview();
	unsigned char fix_Quality();
	bool encode(char c);
	void get_position(long *latitude, long *longitude, unsigned long *fix_age);
	void get_datetime(unsigned long *date, unsigned long *time, unsigned long *age);
	void crack_datetime(int *year, byte *month, byte *day, 
						byte *hour, byte *minute, byte *second, byte *hundredths, unsigned long *age);
	
private:
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

	// methods
	int from_hex(char a);
	unsigned long parse_decimal();
	unsigned long parse_degrees();
	bool term_complete();
	long gpsatol(const char *str);
	int gpsstrcmp(const char *str1, const char *str2);
	bool gpsisdigit(char c);
};
