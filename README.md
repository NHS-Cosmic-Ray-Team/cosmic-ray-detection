# cosmic-ray-detection

Code for a cosmic ray telescope, as provided by the Pratt Institute under the direction of Dr. Helio Takai. The cosmic ray telescope is powered by an Arduino and records data on a Raspberry Pi, which uploads it to Dropbox. The code was mostly written by some grad student who didn't necessarily document it well, so this repository seeks to fix that.

In addition to cosmic rays, the base Arduino sketch also measures:
* Ambient weather conditions (temperature, humidity, pressure) using a BME280 sensor and [Aadafruit's BME280 library](https://github.com/adafruit/Adafruit_BME280_Library)
* GPS coordinates using a modified version of the [TinyGPS library](https://github.com/mikalhart/TinyGPS)

*__NOTE:__ you are not required to import these libraries, since they are included in the code to save memory.*

It is strongly encouraged that you use this code as a basis on which you add other sensors for other experiments. The base Arduino sketch comes in two forms:
* *CombinedCodeNoLibrary* - the original code, as provided by the Pratt Institute. The aforementioned libraries are in the code to save memory.
* *BaseCode* - the same thing, but the libraries were moved to a compact custom library included with the sketch for readability purposes.

The *Northport* Arduino sketch example of this code being expanded. It is used for an experiment at Northport High School where an investigation on the effects of CO2, Ozone, and the Earth´s magnetic field on cosmic rays is conducted. The following was added on top of the base code:
* [SparkFun MAG3110 magnetometer](https://www.sparkfun.com/products/retired/12670)
	* [Library](https://github.com/sparkfun/SparkFun_MAG3110_Breakout_Board_Arduino_Library)
* [ControlEverything ADC121C_MQ131 ozone gas sensor](https://store.ncd.io/product/mq131-ozone-gas-sensor-adc121c-12-bit-adc-i%C2%B2c-mini-module/)
	* There is no library, but the code is heavily based on [sample code](https://github.com/ControlEverythingCommunity/ADC121C_MQ131) that ControlEverything provides
* [DFRobot analog CO2 sensor](https://www.dfrobot.com/product-1549.html)