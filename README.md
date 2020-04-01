# SlowControl
A library that allow you to communicate to Slow Control Board designed by UCLouvain.

# Example
The library comes from a basic Sketch. See File > Examples > SlowControl within the Arduino application.

# Compatible Hardware
The library uses the ESP8266 WiFi Client. This means it will only Works for ESP-Board at least for communication.
Actually, the library is specifially writed to be compatible with our designed board :

- DiSCo Board

# What does the library include ?

- First automated wifi connection creating Acces Point to enter WiFi Informations || Take previous credentials if it has already connected.
- MQTT Communication Management || Automatically reconnecting when server is reachable.
- Sensors readings independent from Network Connection.
- Event Status Update of others DiSCo Board based on MQTT Subscription.
