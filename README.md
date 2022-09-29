# SlowControl

A library that allow you to communicate to the DiscO slow control board designed by UCLouvain.
It is meant to work with the [DiscO_Sensors library](https://cp3-git.irmp.ucl.ac.be/phase2tracker/disco_sensors) for reading out 
the board with the DS18B20 and BME280 sensors, or for reading out the SCD30 sensor.
Which sensors to read can be easily configured in the sketch.

# Example and configuration

The library comes with a basic sketch, see [examples/slowcontrol_basic/slowcontrol_basic.ino](examples/slowcontrol_basic/slowcontrol_basic.ino).

See the `#define` directives at the top of that sketch to configure the active sensors, the board ID, ...

# What does the library include ?

- Automated wifi connection creating an access point to enter WiFi Informations (but take previous credentials if it was connected before)
- MQTT Communication Management (automatically reconnecting when server is reachable)
- Sensors readings independent from network connection.
- Currently disabled: ~~Event Status Update of others DiSCo Board based on MQTT Subscription.~~

# Library dependencies

The libraries should be installed through the Arduino IDE, make sure you use the ones below:

- Board manager: https://arduino-esp8266.readthedocs.io/
- For the sensors:
    - DS18B20: https://github.com/milesburton/Arduino-Temperature-Control-Library
    - BME280: https://github.com/sparkfun/SparkFun_BME280_Arduino_Library
    - CCS811: https://github.com/sparkfun/SparkFun_CCS811_Arduino_Library
    - SCD30: https://github.com/sparkfun/SparkFun_SCD30_Arduino_Library
    - 1-wire: https://github.com/PaulStoffregen/OneWire
- https://arduinojson.org/
- https://github.com/mike-matera/ArduinoSTL
- https://github.com/tzapu/WiFiManager
- https://github.com/knolleary/pubsubclient
