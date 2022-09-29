#include <Disco_Sensors.h>
#include <SlowControl.h>

//////////////////////////////
/// General configuration ////
//////////////////////////////

#define DISCO_ID "ESP03"

// Set MQTT Server - Uncomment this definition if you want to hardcode your MQTT
// Credentials - If not you will be able to enter it in the Access Point
//#define DEFAULT_MQTT_SERVER "192.168.1.105"

#define PUB_INTERVAL_MS 5000  // polling & publication interval in ms


// Active sensors, comment to deactivate
//#define SENSOR_DS  // DS18B20: temperature, precise
//#define SENSOR_BME  // BME280: humidity, temperature, pressure
//#define SENSOR_CCS  // CCS811: VOC
#define SENSOR_SCD  // SCD30: CO2

//////////////////////////////
/// End configuration ////////
//////////////////////////////


SlowControl slowControl(DISCO_ID);
// Some shenanigans because the DS/BME/CCS board and the SCD sensor
// don't use the same ESP pin for I2C SDA, for some reason...
#ifdef SENSOR_SCD
Disco_Sensors mySensors(2, 14);
#elif defined(SENSOR_BME) || defined(SENSOR_CCS)
Disco_Sensors mySensors(13, 14);
#else
// DS sensor only on 1-wire, don't care about I2C
Disco_Sensors mySensors;
#endif

// To RESET WiFi Settings -- Send CMD "RESET@" via Serial

void setup() {
  // Start Slow Control Library
  slowControl.begin();

  // Start Disco Sensors Library
  #ifdef SENSOR_DS
  mySensors.activateDS();
  #endif
  #ifdef SENSOR_BME
  mySensors.activateBME();
  #endif
  #ifdef SENSOR_CCS
  mySensors.activateCCS();
  #endif
  #ifdef SENSOR_SCD
  mySensors.activateSCD();
  #endif
  mySensors.begin();

  // Reset WiFi Settings if needed --> Uncomment --> Upload --> Comment Again
  // slowControl.resetWifiSettings();

  // Initialize WiFi Communication
  slowControl.connectToWifi();

  #ifdef DEFAULT_MQTT_SERVER
  slowControl.setMQTTServer(DEFAULT_MQTT_SERVER);
  #endif

  // Initialize MQTT Communication
  slowControl.connectToMQTT(3, false); // Parameter 1 = Number of tries
                                       // Parameter 2 = Subscription to TTl Status

  // Set Sensors values to trigger alarms
  // mySensors.setAlarmSensor("TEMP", 14.0, 27.0);

  // mySensors.setAlarmSensor("HUMI",3.0,5.0);

  // mySensors.setAlarmSensor("CO2",3.0,5.0);

  // mySensors.setAlarmSensor("TVOC",3.0,5.0);
}

void loop() {
  // Monitor WiFi & MQTT Reconnection
  slowControl.run();

  // Provide Disco Board readings from sensors
  auto sensor_values = mySensors.getReadings();

  // Print values from sensors
  mySensors.printValues(sensor_values);

  // Check Alarms set by the User for the sensor
  /* if (mySensors.checkSensorsTreshold()) {
    slowControl.set_TTL_OUTPUT(true);
  } else {
    slowControl.set_TTL_OUTPUT(false);
  } */

  // Check if Disco is connected to MQTT_Server
  if (slowControl.isConnected()) {
    // Publish Values
    slowControl.publishValues(sensor_values);
  }
  
  delay(PUB_INTERVAL_MS);
}
