/*

mag_calibrate.ino

Calibrates a MAG3110 magnetometer and writes calibration data
to EEPROM to be retrieved later. A speaker on pin 2 is used to
indicate the magnetometer is calibrated

Author: Kyle Mitard
Created 18 December 2019


*/

#include <SparkFun_MAG3110.h>
#include <Wire.h>
#include <ADC121C_MQ131.h>
#include <EEPROM.h>

MAG3110 mag = MAG3110();
ADC121C_MQ131 ozone = ADC121C_MQ131();

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  mag.initialize();
  
  if (mag.error)
  {
    Serial.println("Error with magnetometer");
    
    while (true)
    {
      delay(1);
    }
  }
  
  mag.start();
  mag.enterCalMode();
  Serial.println("calibrating...");
  while (!mag.isCalibrated())
  {
    mag.calibrate();
    delay(100);
  }
  mag.exitCalMode();
  
  Serial.println("Done!");
  Serial.println(mag.readOffset(MAG3110_X_AXIS));
  Serial.println(mag.readOffset(MAG3110_Y_AXIS));
  
  int xOffset = mag.readOffset(MAG3110_X_AXIS);
  int yOffset = mag.readOffset(MAG3110_Y_AXIS);
  
  writeNum(xOffset, 4);
  writeNum(yOffset, 7);
  Serial.println(readNum(4));
  
  //a speaker on pin 2 will sound to indicate calibration
  tone(2, 440);
}

void loop()
{
  delay(10);
  if (mag.readHeading() < 3.0)
  {
    //a higher pitch will indicate correct orientation
    tone(2, 880);
  }
  else
  {
    tone(2, 440);
  }
  
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

//reads a calibration number from the EEPROM as it was written
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
