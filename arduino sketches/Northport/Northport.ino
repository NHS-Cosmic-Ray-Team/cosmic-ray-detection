/*
  Northport.ino

  The cosmic ray telescope code, modified to add an analog carbon dioxide sensor, MAG3110
  magnetometer, and ADC121C_MQ131 Ozone sensor for research being performed at Northport
  High School.

  Authors: TK Doe, Joe Sundermier, Kyle Mitard
  Created  11 November 2019

  Requires the SparkFun_MAG3110 library
  https://github.com/sparkfun/SparkFun_MAG3110_Breakout_Board_Arduino_Library



Reads the muons coincidence from encoder, RMC and GGA sentences from GPS shield and data from BME 
sensor at a minute intervals (can be changed to anytime), using the GPS as a timer. 
The analog input voltages supplied to the scintillators are also read in and monitored. 
We also calculate the latency between the satelites and our GPS sensor.Data is written to the 
serial monitor to be saved on the raspberry pi. The counter (discriminator shield) is connected 
to two scintilators configured as a cosmic ray telescope.

NOTE: There's no longer a neeed to import the GPS and BME280 libraries.
      The needed functions from these libraries are included in the
	    CosmicRayExtras library included with this code.

Timer: To test whether the TinyGPS object contains valid fix data, pass the address of an unsigned 
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

#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include "CosmicRayExtras.h"

#include "ADC121C_MQ131.h"
#include "SparkFun_MAG3110.h"
#include <EEPROM.h>

// weather sensor
myBME280 bme;

// GPS unit
myGPS gps;

SoftwareSerial ss(8, 7);

// data from GPS
unsigned long gpsDate,  // date in the form ddmmyy
              gpsTime,  // time in the form hhmmsscc (retrieved from the GPS, not the Pi, so make sure they are in PERFECT sync with each other!)
              gpsAge;   // age of GPS data in milliseconds


// latitude and longitude in millionths of a degree (this is how the GPS outputs it)
long int  lat, lon;

// (unsure) length of time in one pulse of data from GPS
unsigned long signal_1pps;

// latency in getting data from GPS
unsigned long latency;

// true if new data was recieved from GPS
bool newData = false;

// number of cosmic rays counted by scintillators
signed long count = 0;

int second = 0,       // don't know
    flag = 0,         // flag to delay 50 ms after reading GPS when a 1PPS signal is sent for some reason?
    lock_minute = -1, // the last minute when data was outputted
    oldSecond = -1;   // don't know

// ozone sensor
ADC121C_MQ131 ozone;

// magnetometer
MAG3110 mag;

// header of file, with every field separated by a comma
const String fileHeader = "Date,Time,Count,TempC,Pressure,Humidity,Lat,Lon,Numsats,FixQ,Latency,CO2,Ozone,magX,magY,magZ";


//---------------------------------------Subroutines-------------------------------------

// read a new set of Data from the GPS
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


// initialize the counter
void initCounter()
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


// Reads the Encoders to retrieve the updated value and RETURNS: long
long readCounter()
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


// prints date in the form (yyyymmdd)
static void print_date(myGPS &gps)
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


// takes all data from everything and prints it in the file
void outputData()
{
  // date/time
  print_date(gps);
  Serial.print(",");

  // cosmic ray counter
  count = readCounter(); //calls subroutine to read the counter
  Serial.print(count);
  Serial.print(",");

  digitalWrite(9, LOW); //begin SPI conversation
  SPI.transfer(0x20); //clears the CNTR
  digitalWrite(9, HIGH); //end SPI conversation

  // weather sensor (temperature, pressure, humidity)
  Serial.print(bme.readTemperature());
  Serial.print(",");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.print(",");
  Serial.print(bme.readHumidity());
  Serial.print(",");

  // latitude and longitude (converted to degrees)
  Serial.print((lat / 1000000.0), 3);// print latitude
  Serial.print(",");
  Serial.print((lon / 1000000.0), 3);
  Serial.print(",");

  // technical GPS stuff
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
  
  //CO2 Sensor
  checkVoltage();

  // ozone sensor
  Serial.print(ozone.get_ppm());
  Serial.print(",");

  // magnetometer
  readMagnetometer();

  // I don't know what this does
  oldSecond = second;
}


// (unsure) sets a flag when a 1PPS signal is sent
void one_pps()
{
  signal_1pps = micros();
  flag = 0;
}


// reads voltage from CO2 sensor and converts it to ppm using a conversion described by the manufacturer
void checkVoltage()
{
  // read the input on analog pin 0:
  int analogValue0 = analogRead(A0);

  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  double voltage0 = analogValue0 * (5.0 / 1023.0);

  // convert analog voltage to CO2 concentration in ppm
  double co2_ppm = (voltage0 - 0.4) * 5000.0 / 1.6;

  // print out the value
  Serial.print(co2_ppm);
  Serial.print(",");
}


// initializes all sensors and such (runs once when Arduino starts up)
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
  Serial.println(fileHeader);
  attachInterrupt(0, one_pps, RISING); //1PPS signal to pin 2, rising signal.

  ozone.begin();
  mag.initialize();
  mag.start();
  importMagCalibrationData();
}


// reads and outputs data (loops indefinitely after setup() is finished)
void loop()
{
  // get data from GPS
  readGPS();

  // date, time, and age of data stored in variables gpsDate, gpsTime, gpsAge
  gps.get_datetime(&gpsDate, &gpsTime, &gpsAge);
  
  // latitude/longitude and age (again for some reason) stored in variables lat, lon, gpsAge
  gps.get_position(&lat, &lon, &gpsAge);

  // from time, get hour and minute
  // ex: if gpsTime = 231700, minute = 17 and hour = 23
  int minute = (gpsTime / 10000) % 100;
  int hour = gpsTime / 1000000;

  // index of data points, signified by the minutes after midnight
  int index = (hour * 60) + minute;

  // if the GPS data is not stale (older than 1 second) and a minute has passed since last data point was written
  if ((gpsAge <= 1000) && (minute != lock_minute))
  {
    lock_minute = minute;

    // if it is midnight (i.e. when a new file is created), print out file header
    // this is supposed to put it at the top of the file, although it only works properly when the Raspberry Pi's clock is in perfect sync with the GPS
    if ((index == 0) )
    {
      Serial.println(fileHeader);
    }

    outputData();
  }
}


//EXTRA MAGNETOMETER METHODS ----------------------------------------------------------------------------

// reads and prints data from magnetometer (B field strength in X, Y, and Z axes in microteslas)
void readMagnetometer()
{
  float x, y, z;
  
  mag.readMicroTeslas(&x, &y, &z);

  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.println(z);
}

// imports calibration data from bytes 4 through 9
void importMagCalibrationData()
{
  int xOffset = readNum(4);
  mag.setOffset(MAG3110_X_AXIS, xOffset);
  
  int yOffset = readNum(7);
  mag.setOffset(MAG3110_Y_AXIS, yOffset);
}

// writes a calibration number to 3 EEPROM slots by treating
// each slot as a digit of a base-256 number system
void writeNum(int n, int firstByte)
{
  
  //first slot: 1 if negative, 0 if positive
  if (n < 0)
  {
    EEPROM.write(firstByte, 1);
  }
  else
  {
    EEPROM.write(firstByte, 0);
  }
  
  EEPROM.write(firstByte + 1, n / 256);
  
  EEPROM.write(firstByte + 2, n % 256);
}

//reads a calibration number from the EEPROM as it was written (see writeNum() above)
int readNum(int firstByte)
{
  int n = 0;
  
  n = n + EEPROM.read(firstByte + 2);
  n = n + 256 * EEPROM.read(firstByte + 1);
  
  //if the first byte is 1, the number is negative
  if (EEPROM.read(firstByte) == 1)
  {
    n = n * -1;
  }
  
  return n;
}
