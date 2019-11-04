/*
	ADC121C_MQ131.h

	Author: Kyle Mitard but not really
	Created 3 Nov 2019

	Header file for an Arduino lobrary that is to be used with the ADC121C_MQ131 ozone sensor.
*/

#ifndef ADC121C_MQ131_h
#define ADC121C_MQ131_h
#endif

class ADC121C_MQ131
{
public:
	ADC121C_MQ131();
	void begin();
	double get_ppm();
private:
	unsigned int data[2];
	int raw_adc;
};