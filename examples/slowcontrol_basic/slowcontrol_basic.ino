#include <Disco_Sensors.h>
#include <SlowControl.h>

// Array to store Readings from sensors
#define ARRAY_SIZE 5
String sensors_values[ARRAY_SIZE];

// Alarms Treshold Definitions
#define DS_TEMP_TRESHOLD_MIN 50.0
#define DS_TEMP_TRESHOLD_MAX 50.0
#define BME_TEMP_TRESHOLD_MIN 50.0
#define BME_TEMP_TRESHOLD_MAX 50.0
#define BME_HUMI_TRESHOLD_MIN 56.0
#define BME_HUMI_TRESHOLD_MAX 56.0
#define CCS_CO2_TRESHOLD_MIN 1600
#define CCS_CO2_TRESHOLD_MAX 1600
#define CCS_TVOC_TRESHOLD_MIN 100
#define CCS_TVOC_TRESHOLD_MAX 100

SlowControl slowControl;

Disco_Sensors mySensors(ARRAY_SIZE);

// To RESET WiFi Settings -- Send CMD "RESET\@" via Serial

void setup() {
  // Initialize Serial Communication
  Serial.begin(115200);

  // Start Slow Control Library
  slowControl.begin();

  // Start Disco Sensors Library
  mySensors.begin();

  // Reset WiFi Settings if needed --> Uncomment --> Upload --> Comment Again
  // --> Upload slowControl.resetWifiSettings();

  // Initialize WiFi Communication
  slowControl.connectToWifi();

  // Set MQTT Server - Uncomment this function if you want to hardcode your MQTT
  // Credentials - If not you will be able to enter it in the Acces Point
  // Configuration slowControl.setMQTTServer("192.168.0.108");

  // Initialize MQTT Communication
  slowControl.connectToMQTT(
      3, "ESP02",
      true); // Parameter 1 = Number of Try  || Parameter 2 = ID of the Board ||
             // Parameter 3 = Subscription to TTl Status

  // Set Sensors values to trigger alarms
  mySensors.setAlarmSensor("TEMP", 14.0, 27.0);

  // mySensors.setAlarmSensor("HUMI",3.0,5.0);

  // mySensors.setAlarmSensor("CO2",3.0,5.0);

  // mySensors.setAlarmSensor("TVOC",3.0,5.0);
}

void loop() {
  // Monitor WiFi & MQTT Reconnection
  slowControl.run();

  // Provide Disco Board readings from sensors
  mySensors.getReadings(sensors_values);

  // Print values from sensors
  mySensors.printValues(sensors_values);

  // Check Alarms set by the User for the sensor
  if (mySensors.checkSensorsTreshold()) {
    slowControl.set_TTL_OUTPUT(true);
  } else {
    slowControl.set_TTL_OUTPUT(false);
  }

  // Check if Disco is connected to MQTT_Server
  if (slowControl.isConnected()) {
    // Publish Values
    slowControl.publishValues(sensors_values);
  }
  delay(2000);
}
