#include <ArduinoSTL.h>
#include <Disco_Linduino.h>
#include <SlowControl.h>

//////////////////////////////
/// General configuration ////
//////////////////////////////

#define DISCO_ID "ESP04"

// channel nr. 1 connected to channel 4 on DC2210A board
#define CHANNELS { {"1", 4}}

// Set MQTT Server - Uncomment this definition if you want to hardcode your MQTT
// Credentials - If not you will be able to enter it in the Access Point
//#define DEFAULT_MQTT_SERVER "192.168.1.105"

#define PUB_INTERVAL_MS 5000 // polling & publication interval in ms

//////////////////////////////
/// End configuration ////////
//////////////////////////////

SlowControl slowControl(DISCO_ID);
Disco_Linduino mySensors(CHANNELS);

// To RESET WiFi Settings -- Send CMD "RESET@" via Serial

void setup() {
  // Start Slow Control Library
  slowControl.begin();

  mySensors.begin();

  // Reset WiFi Settings if needed --> Uncomment --> Upload --> Comment Again
  // slowControl.resetWifiSettings();

  // Initialize WiFi Communication
  slowControl.connectToWifi();

#ifdef DEFAULT_MQTT_SERVER
  slowControl.setMQTTServer(DEFAULT_MQTT_SERVER);
#endif

  // Initialize MQTT Communication
  slowControl.connectToMQTT(3,
                            false); // Parameter 1 = Number of tries
                                    // Parameter 2 = Subscription to TTl Status
}

void loop() {
  // Monitor WiFi & MQTT Reconnection
  slowControl.run();

  // Provide Disco Board readings from sensors
  auto sensor_values = mySensors.getReadings();

  // Print values from sensors
  mySensors.printValues(sensor_values);

  // Check if Disco is connected to MQTT_Server
  if (slowControl.isConnected()) {
    // Publish Values
    slowControl.publishValues(sensor_values);
  }

  delay(PUB_INTERVAL_MS);
}
