/***************************************************
  This is a library for Slow Control System

  Designed specifically to work with the Slow Control Board from UCLouvain
 designed by Antoine Deblaere

  This library will allow you to easily connect to WIFI, communicate with
 Sensors, and more


  Written by Antoine Deblaere for IRMP/CP3
  Modified by Sebastien Wertz
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#ifndef SlowControl_H
#define SlowControl_H

#include "Arduino.h"
#include "ESP8266WiFi.h"
#include <ArduinoJson.h>
#include <DNSServer.h> //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h> //Local WebServer used to serve the configuration portal
#include <PubSubClient.h>
#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager WiFi Configuration Magic

//#define SLOWCONTROL_DEFAULT_MQTT_SERVER "DESKTOP-09R46K7.local"
#define SLOWCONTROL_DEFAULT_MQTT_SERVER ""
#define SLOWCONTROL_DEFAULT_MQTT_PORT 1883
#define SLOWCONTROL_DEFAULT_MQTT_CLIENT_ID "SlowControlBoard"
#define SLOWCONTROL_DEFAULT_NBR_OF_TRY 5
#define SLOWCONTROL_DEFAULT_TTL 16

#define statusTTL_topic                                                        \
  "/status/ttl/" // Will use the ID of the Board to retrieve where the status
                 // come from

typedef std::vector<std::vector<String>> SensorValues;

class SlowControl {
public:
  /**
   *  Constructor.
   */
  SlowControl(const String &clientID = SLOWCONTROL_DEFAULT_MQTT_CLIENT_ID);

  /**
   * Initialises the Slow Control Library
   */
  void begin();

  /**
   * Start running of the Slow Control Library
   */
  void run();

  /**
   * Set the MQTT Server harcoded
   *
   * Take parameter of the function and provides it to the MQTT Client
   *
   *If not create you will be able to enter it in the Access Point while
   *configuring the Wifi
   */
  void
  setMQTTServer(const String &mqtt_server = SLOWCONTROL_DEFAULT_MQTT_SERVER,
                uint16_t mqtt_port = SLOWCONTROL_DEFAULT_MQTT_PORT);

  /**
   * Initialises the WIFI Communication
   *
   * Take previous credentials if the board has already connected to Wifi
   *
   *If not create an Wifi AP to enter the credentials of our personal Network
   */
  void connectToWifi();

  /**
   * Reset WiFi Settings
   *
   * In order to change WiFi Credentials, you can call this function to perform
   * a reset of them
   *
   */
  void resetWifiSettings();

  /**
   * Initialises the MQTT Communication
   */
  void connectToMQTT(int nbr = SLOWCONTROL_DEFAULT_NBR_OF_TRY,
                     bool connectToTTL = false);

  /**
   * Check if MQTT is connected
   */
  bool isConnected();

  /**
   * Publish to MQTT Topic
   *
   *Param 1 : Topic where you want to publish || Param 2 : Data you want to
   *publish
   *
   */
  void publishToMQTT(const String &topic, const String &payload);

  /**
   * Publish values to MQTT Server
   *
   *Param 1 : Topic where you want to publish || Param 2 : Data you want to
   *publish
   *
   */
  void publishValues(const SensorValues &data);

  /**
   * Subscribe to TTL Status
   *
   *Take default TTL Topic as paramaters
   *
   *If it doesnt suit, pass yours as paramaters
   */
  void subscribeToSCBStatus(const String &topic = statusTTL_topic);

  /**
   * Setup callback function
   *
   * Get feeback from topic that changed if you subscribed to them
   */
  void callbackTTL(char *topic, byte *payload, unsigned int length);

  /**
   * Trigger TTL
   */
  void set_TTL_OUTPUT(int state);

private:
  /**
   *  Constructor.
   */
  WiFiManager _wifiManager;
  WiFiClient _espClient;
  PubSubClient _mqttClient;

  /**
   *  Status of Sensors Variables.
   */
  String _clientID;
  bool _ttlStatus;
  bool _connectToTTL;
  String _mqttServer;
  uint16_t _mqttPort;
  bool _mqttServerSet;
  static bool _shouldSaveConfig;

  /**
   *  Private Functions
   */
  String byteArrayToString(byte *payload, unsigned int length);

  String getValue(String data, char separator, int index);

  void receiveWithEndMarker();
  void showNewData();

  /**
   *
   */
  const static byte _numChars = 32;
  char _receivedChars[_numChars]; // an array to store the received data
  bool _newData = false;

  /**
   * Check JSON Config File
   *
   *
   *
   */
  void checkJSONConfig();

  static void saveConfigCallback();
};

#endif
