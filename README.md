# cosmic-ray-detection

Code for a cosmic ray telescope, as provided by the Pratt Institute under the direction of Dr. Helio Takai. The cosmic ray telescope is powered by an Arduino and records data on a Raspberry Pi, which uploads it to Dropbox. The code was mostly written by some grad student who didn't necessarily document it well, so this repository seeks to fix that.

In addition to cosmic rays, the base Arduino sketch also measures:
* Ambient weather conditions (temperature, humidity, pressure) using a BME280 sensor and [Aadafruit's BME280 library](https://github.com/adafruit/Adafruit_BME280_Library)
* GPS coordinates using a modified version of the [TinyGPS library](https://github.com/mikalhart/TinyGPS)

*__NOTE:__ you are not required to import these libraries, since they are included in the code to save memory.*

It is strongly encouraged that you use this code as a basis on which you add other sensors for other experiments. The base Arduino sketch comes in two forms:
* *CombinedCodeNoLibrary* - the original code, as provided by the Pratt Institute. The aforementioned libraries are in the code to save memory.
* *BaseCode* - the same thing, but the libraries were moved to a compact custom library included with the sketch for readability purposes.