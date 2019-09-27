/*@authors - TK Doe, Joe Sundermier, Kyle Mitard
  @date - September 26, 2019
  @version - most updated code

This is a modified version of CombinedCodeNoLibrary.ino which now uses
external libraries again, since the apparent issues with memory do not
seem to exist, although I could very much be wrong (if you're paranoid
just use an Arduino Mega)

Reads the muons coincidence from encoder, RMC and GGA sentences from GPS shield and data from BME 
sensor at a minute intervals (can be changed to anytime), using the GPS as a timer. 
The analog input voltages supplied to the scintillators are also read in and monitored. 
We also calculate the latency between the satelites and our GPS sensor.Data is written to the 
serial monitor to be saved on the raspberry pi. The counter (discriminator shield) is connected 
to two scintilators configured as a cosmic ray telescope.

Libraries used:
    Adafruit_BME280    https://github.com/adafruit/Adafruit_BME280_Library
    TinyGPS (SEE NOTE) https://github.com/mikalhart/TinyGPS
    Adafruit_GPS       https://github.com/adafruit/Adafruit_GPS
    Adafruit_sensor    https://github.com/adafruit/Adafruit_Sensor

NOTE: The TinyGPS library used was modified when there were no libraries, so it is
      required that you use the modified version of the library that is in the cosmic
      ray detection GitHub: https://github.com/muonmasters/cosmic-ray-detection

Timer: To test whether the TinyGPS_modded object contains valid fix data, pass the address of an unsigned 
       long variable for the “fix_age” parameter in the methods that support it. 
       If the returned value is GPS_INVALID_AGE, then you know the object has never received a valid fix. 
       If not, then fix_age is the number of milliseconds since the last valid fix. 
       Since we are feeding the object regularly, fix_age should NOT get much over 1000. If fix_age starts getting large, 
       that may be a sign that you once had a fix, but have lost it.

       To get the best results, we first check to validate that the GPS has a fix before any data is logged
       
-------------------------------------------------------------------------------
-------------------------------------------------------------------------------
 Avoid pin conflict || Please follow all connection/pin configurations in setup
-------------------------------------------------------------------------------
SPI connect counter
I2C connect BME280
   
            MOSI   -------------------   SDO (D11)
            MISO   -------------------   SDI (D12)
            SCK    -------------------   SCK (D13)
            SS1    -------------------   SS1 (D9)
            GND    -------------------   GND
            VDD    -------------------   VCC (5.0V)
            A(12)  -------------------   Signal
            B(11)  -------------------   tie to logic high for up count
            I(10)  -------------------   tie to +V
            A(0)   -------------------   Analog Voltage 1
            A(1)   -------------------   Analog Voltage 2
            D(2)   -------------------   1PPS on GPS Shield
            
This is probably not necessary since the LS7366 starts up in this mode:
configure MDR0 into non-quadrature, free runnig mode  0000 0000 --> 0x00
configure MDR1                                        0000 0000 --> 0x00

*/
#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Adafruit_BME280.h>
#include <TinyGPS_modded.h>


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


Adafruit_BME280 bme; // I2C
TinyGPS_modded gps;
SoftwareSerial ss(8, 7);

unsigned long gpsDate, gpsTime, gpsAge;
float flat, flon;
long int  lat, lon;
unsigned long signal_1pps;
unsigned long latency;

bool newData = false;
signed long count = 0;
int second = 0,flag = 0, lock_minute = -1, oldSecond = -1, oldMinute = -1;
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

static void print_date(TinyGPS_modded &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
    char sz[36];
    sprintf(sz, "%02d%02d%02d,%02d%02d%02d.%02d",
        year, month, day, hour, minute, second, hundredths);
    Serial.print(sz);
}

void outputData()
{
  print_date(gps);
  Serial.print(",");

  count = readCounter(); //calls subroutine to reac the counter
  Serial.print(count);
  Serial.print(",");

  digitalWrite(9, LOW); //begin SPI conversation
  SPI.transfer(0x20); //clears the CNTR
  digitalWrite(9, HIGH); //end SPI conversation

  barometer();

  Serial.print(",");
  Serial.print((lat / 1000000.0), 3);// print latitude
  Serial.print(",");
  Serial.print((lon / 1000000.0), 3);
  Serial.print(",");
  Serial.print(gps.satsinview());
  Serial.print(",");
  Serial.print(gps.fix_Quality());
  Serial.print(",");
  Serial.print((latency / 1000.0));
  if ((latency / 1000.0) > 350) // this is to check if the GPS resets to 1Hz update rate and sets it back to the desired 5Hz update rate
  {
      ss.print(GPS_RMC);
      ss.print(GPS_RMCGGA);
      ss.print(GPS_5Hz);
  }
  Serial.print(",");
  checkVoltage();

  oldSecond = second;
}

void one_pps()
{
  signal_1pps = micros();
  flag = 0;
}

void checkVoltage() // Analog to Digital Converter
{
  // read the input on analog pin 0:
  int analogValue1 = analogRead(A0);
  int analogValue2 = analogRead(A1);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage1 = analogValue1 * (5.0 / 1023.0);
  float voltage2 = analogValue2 * (5.0 / 1023.0);
  // print out the value you read:
  Serial.print(voltage1);
  Serial.print(",");
  Serial.println(voltage2); 
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
  Serial.println("Date,Time,Count,TempC,Pressure,Humidity,Lat,Lon,Numsats,FixQ,Latency,V1,V2");
  attachInterrupt(0, one_pps, RISING); //1PPS signal to pin 2, rising signal.
}

void loop()
{
  readGPS();
  gps.get_datetime(&gpsDate, &gpsTime, &gpsAge);
  gps.get_position(&lat, &lon, &gpsAge);
  int minute = (gpsTime / 10000) % 100;
  int hour = gpsTime / 1000000;
  int index = (hour * 60) + minute;

  if (gpsAge <= 1000) //To test whether the data returned is stale: fix_age returns the number of milliseconds since the data was encoded.
  {
    if ((index == 0) && (minute != lock_minute)) //check that it's midnight
    {
      lock_minute = minute;
      Serial.println("Date,Time,Count,TempC,Pressure,Humidity,Lat,Lon,Numsats,FixQ,Latency,V1,V2");
      outputData();
    }
    if ((index % 1 == 0) && (minute != lock_minute)) //check that minute is 5
    {
      lock_minute = minute;
      outputData();
    }
  }
}
