# SlowControl

A library that allow you to communicate to the DiscO slow control board designed by UCLouvain.
It is meant to work with the [DiscO_Sensors library](https://cp3-git.irmp.ucl.ac.be/phase2tracker/disco_sensors) for reading out 
the board with the DS18B20 and BME280 sensors, for reading out the SCD30 sensor, or for interfacing with a Linduino board itself reading out
analog temperature sensors through and ADC.
Which sensors to read can be easily configured in the sketch.
For the firmware of the Linduino board itself, see https://cp3-git.irmp.ucl.ac.be/phase2tracker/lindui2983rtd

# Example and configuration

The library comes with two example sketches:
- one to handle the various digital sensors (DS18, BME280, SCD30...), see [examples/slowcontrol_basic/slowcontrol_basic.ino](examples/slowcontrol_basic/slowcontrol_basic.ino).
- one to handle the Linduino board interfacing with an ADC for analog temperature sensor (RTD) readout, see [examples/slowcontrol_linduino/slowcontrol_linduino.ino](examples/slowcontrol_linduino/slowcontrol_linduino.ino).

See the `#define` directives at the top of the sketches to configure the active sensors, the board ID, ...

For flashing the firmware, install the board manager listed below and select "Generic ESP8266 Module". The serial monitor works with 115200 baud.

# What does the library include ?

- Automated wifi connection creating an access point to enter WiFi Informations (but take previous credentials if it was connected before)
- MQTT Communication Management (automatically reconnecting when server is reachable)
- Sensors readings independent from network connection.
- Currently disabled: ~~Event Status Update of others DiSCo Board based on MQTT Subscription.~~

# Library dependencies

The libraries can be installed through the Arduino IDE, make sure you use the ones below:

- Board manager: see https://arduino-esp8266.readthedocs.io/ for installation
    - esp8266 by ESP8266 community, tested with 3.1.0
- For the sensors:
    - DS18B20: DallasTemperature by Miles Burton, tested with v3.8.0, https://github.com/milesburton/Arduino-Temperature-Control-Library
    - BME280: Sparkfun BME280 by Sparkfun, tested with v2.0.8, https://github.com/sparkfun/SparkFun_BME280_Arduino_Library
    - CCS811: Sparkfun CCS811 by Sparkfun, tested with v2.0.2, https://github.com/sparkfun/SparkFun_CCS811_Arduino_Library
    - SCD30: Sparkfun SCD30 by Sparkfun, tested with v1.0.19, https://github.com/sparkfun/SparkFun_SCD30_Arduino_Library
- ArduinoJson by Benoit Blanchon, tested with v6.19.4, https://arduinojson.org/
- ArduinoSTL by Mike Matera, tested with v1.3.2, https://github.com/mike-matera/ArduinoSTL
- OneWire by Jim Studt et al, tested with v2.3.4, https://github.com/PaulStoffregen/OneWire
- WiFiManager by tzapu, tested with 2.0.14-beta, https://github.com/tzapu/WiFiManager
- PubSubClient by Nick O'Leary, tested with v2.8.0, https://github.com/knolleary/pubsubclient
