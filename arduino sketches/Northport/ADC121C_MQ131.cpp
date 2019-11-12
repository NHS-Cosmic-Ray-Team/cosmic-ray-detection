/*
	ADC121C_MQ131.h
	
	An Arduino lobrary that is to be used with ControlEverything's ADC121C_MQ131 ozone sensor.

	Author: Kyle Mitard but not really
	Created 3 Nov 2019

	This is based off of ControlEverything's sample Arduino sketch, which I decided to make into
	a library, which hadn't been done for some reason. I HAVE NO IDEA HOW THIS WORKS I'M JUST
	PACKAGING SOMEBODY ELSE'S CODE INTO A LIBRARY TO MAKE THINGS EASIER. I TAKE ZERO CREDIT FOR
	THE FUNCTIONALITY OF THIS LIBRARY

	their sample code:
	https://github.com/ControlEverythingCommunity/ADC121C_MQ131/blob/master/Arduino/ADC121C_MQ131.ino
*/

#include "ADC121C_MQ131.h"
#include "Arduino.h"
#include "Wire.h"

//I2C address of the sensor
#define Addr 0x50

//data from sensor in 2 bytes
unsigned int data[2];

//data from the sensor converted into 12 bits
int raw_adc;


/*
* Constructor
*/
ADC121C_MQ131::ADC121C_MQ131()
{
	int a = 2 + 2; //I must have something in here I think
}

/*
* Initialize the sensor... ?
*/
void ADC121C_MQ131::begin()
{
	Wire.begin();
}

/*
* Reads data from the sensor over I2C
* 
* Return: Concentration of Ozone in ppm
*
* I HAVE NO IDEA WHAT IS HAPPENING HERE, I TAKE ZERO CREDIT FOR THE FUNCIONALITY OF THIS LIBRARY.
* NEARLY EVERYTHING HERE IS COPY/PASTED FROM CONTROLEVERYTHING'S SAMPLE CODE
*/
double ADC121C_MQ131::get_ppm()
{
	// Start I2C Transmission
	Wire.beginTransmission(Addr);
	// Select data register
	Wire.write(0x00);
	// Stop I2C transmission
	Wire.endTransmission();

	// Request 2 bytes of data
	Wire.requestFrom(Addr, 2);

	// Read 2 bytes of data
	if (Wire.available() == 2)
	{
		data[0] = Wire.read();
		data[1] = Wire.read();
	}

	// Convert the data to 12-bits
	raw_adc = ((data[0] & 0x0F) * 256) + data[1];

	// Convert data into parts per million of ozone
	return (1.99 * raw_adc) / 4095.0 + 0.01;
}