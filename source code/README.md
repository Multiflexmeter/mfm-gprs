# Changelog

## V1.0
- code made for MFM-Bangladesh, mission February 2019
## V1.1
Some tweaks have been done during the mission in Bangladesh in February 2019
- Increased time to connect to network to give the sensor more time to connect.
- Increase median filter measurements from 9 to 15 to increase accuracy.
- Make URL variable instead of hardcoded. Also modified the SIM7600 library for this.
## V1.2
- Add non equidistant measurement.
- Removed time to send functionality.
## V1.3
- Changes have been made to work with new portal
## V1.4
- Add support for battery voltage measurement
- Add support for temperature measurement DS18B20 (only 1 will be read out for now).
## V1.4.1
- Support for MightyCore.
- Support for flash programming new sketch through FTDI programmer.
- Small improvements to Fona library.
- Improve more than 16-bit integer of time for max interval.

## To do:
- Clean up serial prints
- Make new library
- Bulletproof sim unlock
- Add RTC support
- NTC support
- Add BME280 waterheight calculation support (also add GPS location -> pressure GET)
