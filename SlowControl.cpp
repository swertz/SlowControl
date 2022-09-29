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

#include "SlowControl.h"

bool SlowControl::_shouldSaveConfig = false;

SlowControl::SlowControl(const String& clientID/* = SLOWCONTROL_DEFAULT_MQTT_CLIENT_ID*/):
    _clientID(clientID),
    _ttlStatus(false),
    _connectToTTL(false),
    _mqttServerSet(false),
    _mqttServer(SLOWCONTROL_DEFAULT_MQTT_SERVER),
    _mqttPort(SLOWCONTROL_DEFAULT_MQTT_PORT) {
  // Pass WiFi Connection parameters to MQTT
  _mqttClient.setClient(_espClient);
}

void SlowControl::begin() {
  // Serial Begin
  Serial.begin(115200);

  // Set TTl as OUTPUT
  pinMode(SLOWCONTROL_DEFAULT_TTL, OUTPUT);
  digitalWrite(SLOWCONTROL_DEFAULT_TTL, LOW);
}

void SlowControl::run() {
  // Check Serial Input
  receiveWithEndMarker();
  showNewData();

  // Check MQTT Connection
  if (!_mqttClient.connected()) {
    connectToMQTT(1, _connectToTTL); // Try to reconnect to MQTT one time.
  }
  _mqttClient.loop();
}

void SlowControl::setMQTTServer(
        const String& mqtt_server,
        uint16_t mqtt_port) {
  if (mqtt_server != "") {
    // Get reference of server
    _mqttServer = mqtt_server;
    _mqttPort = mqtt_port;

    // Setup MQTT Server
    _mqttClient.setServer(_mqttServer.c_str(), _mqttPort);

    // Setup Callback
    _mqttClient.setCallback(
        ([this](char *topic, byte *payload, unsigned int length) {
          this->callbackTTL(topic, payload, length);
        }));

    // Status of set
    _mqttServerSet = true;
  } else {
    Serial.println("MQTT Server not hardcoded");
  }
}

void SlowControl::checkJSONConfig() {
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("Mounted File System");
    if (SPIFFS.exists("/config.json")) {
      Serial.println("Reading Config File");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("Opened Config File");

        // need to pass number of bytes expected to be read
        StaticJsonDocument<JSON_OBJECT_SIZE(2) + 50> doc;
        DeserializationError err = deserializeJson(doc, configFile);
        if (!err) {
          serializeJsonPretty(doc, Serial);
          Serial.println();
          _mqttServer = doc["mqtt_server"].as<const char*>();
          _mqttPort = doc["mqtt_port"];
        } else {
          Serial.println("Failed to load Json Config, error is:");
          Serial.println(err.c_str());
        }
      }
    } else {
      Serial.println("JSON File doesn't exist");
    }

  } else {
    Serial.println("Failed to mount FS");
  }
}
void SlowControl::saveConfigCallback() {
  Serial.println("Should Save Config");
  _shouldSaveConfig = true;
}

void SlowControl::connectToWifi() {
  // Check Json Config File
  checkJSONConfig();

  // if at this point the server is empty, something wrong happened
  // and we should make sure we can reconfigure it through the wifi portal
  if (_mqttServer == "") {
      resetWifiSettings();
  }

  // Set Config Save Notify Callback
  _wifiManager.setSaveConfigCallback(&SlowControl::saveConfigCallback);

  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", _mqttServer.c_str(),
                                          40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", String(_mqttPort).c_str(), 6);

  // Check If MQTT Credentials have been hardcoded
  if (!_mqttServerSet) {
    // wifiManager.setBreakAfterConfig(true); //Try to connect one time and
    // thens exits.
    _wifiManager.addParameter(&custom_mqtt_server);
    _wifiManager.addParameter(&custom_mqtt_port);
  }

  // Start Access Point Configuration if you never connected the board to a WiFi
  // previously
  if (!_wifiManager.autoConnect(("AutoConnectAP-" + _clientID).c_str())) {
    Serial.println(F("Failed to connect, we should reset and see if it connects"));
    delay(3000);
    ESP.reset();
    delay(5000);
  }

  Serial.println("Connected to WIFI");
  delay(1000);

  // Print MAC ADRESS
  Serial.println(WiFi.macAddress());

  // Get value
  if (!_mqttServerSet) {
    _mqttServer= custom_mqtt_server.getValue();
    _mqttPort = String(custom_mqtt_port.getValue()).toInt();
  }

  // Save to FS
  if (_shouldSaveConfig) {
    Serial.println("Saving Config");
    StaticJsonDocument<JSON_OBJECT_SIZE(3)> doc;
    doc["mqtt_server"] = _mqttServer;
    doc["mqtt_port"] = _mqttPort;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println(F("Failed to open config file for writing"));
    }

    serializeJsonPretty(doc, Serial);
    Serial.println();
    serializeJson(doc, configFile);
    configFile.close();
    _shouldSaveConfig = false;
  }
}

void SlowControl::resetWifiSettings() {
  _mqttServerSet = false;
  _wifiManager.resetSettings();
}

void SlowControl::connectToMQTT(int nbr, bool connectToTTL) {
  if (_mqttServer != "") {
    setMQTTServer(_mqttServer, _mqttPort);
  }
  _connectToTTL = connectToTTL;
  // Try to connect to MQTT Server n times | 5 S timeout
  for (int i = 0; i < nbr; i++) {
    Serial.println("Attempting MQTT Connection..." + String(i + 1) + "/\0" +
                   String(nbr) + " targeting -> " + _mqttServer + ":" + String(_mqttPort));
  
    if (_mqttClient.connect(_clientID.c_str())) {
      Serial.println("Connected to MQTT Server.");
      Serial.println();
  
      if (_connectToTTL == true) {
        subscribeToSCBStatus();
        Serial.println(F("Subscribed to Slow Control Board Status"));
        Serial.println();
      } else {
        Serial.println(F("You didn't subscribe to Slow Control Board Status"));
        Serial.println();
      }
      break;
    } else {
      delay(5000);
    }
  }
}

bool SlowControl::isConnected() {
  if (_mqttClient.connected()) {
    return true;
  } else {
    return false;
  }
}

void SlowControl::publishToMQTT(const String& topic, const String& payload) {
  _mqttClient.publish(topic.c_str(), payload.c_str());
}

void SlowControl::publishValues(const SensorValues& data) {
  for (const auto& reading: data) {
    publishToMQTT(_clientID + reading[0], reading[1]);
  }
}

void SlowControl::subscribeToSCBStatus(const String& topic) {
  _mqttClient.subscribe(topic.c_str());
}

void SlowControl::callbackTTL(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived");
  String myMsg;
  Serial.println(topic);
  if (String(topic) == statusTTL_topic) {
    myMsg = byteArrayToString(payload, length);
    Serial.println(myMsg);
  } else {
  }
}

void SlowControl::set_TTL_OUTPUT(int state) {
  digitalWrite(SLOWCONTROL_DEFAULT_TTL, state);
}


// Private Fuctions

String SlowControl::byteArrayToString(byte *payload, unsigned int length) {
  String myMsg;

  for (int i = 0; i < length; i++) {
    myMsg += (char)payload[i];
  }
  return myMsg;
}

void SlowControl::receiveWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '@';
  char rc;

  while (Serial.available() > 0 && _newData == false) {
    rc = Serial.read();

    if (rc != endMarker) {
      _receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= _numChars) {
        ndx = _numChars - 1;
      }
    } else {
      _receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      _newData = true;
    }
  }
}

void SlowControl::showNewData() {
  if (_newData == true) {
    String msg { _receivedChars };
    Serial.println(("Received: " + msg).c_str());
    if (msg == "RESET") {
      resetWifiSettings();
      ESP.reset();
    }
    _newData = false;
  }
}

/*void SlowControl::getDataFromEEPROM()
{
        int myData;

        //Get First IP
        firstIP=EEPROMReadInt(adressMQTT);
        Serial.println(firstIP);

        //Get Second IP
        secondIP=EEPROMReadInt(adressMQTT+2);
        Serial.println(secondIP);

        //Get Third IP
        thirdIP=EEPROMReadInt(adressMQTT+4);
        Serial.println(thirdIP);

        //Get Fourth IP
        fourthIP=EEPROMReadInt(adressMQTT+6);

        Serial.println(fourthIP);

        //Get Port
        myPort=EEPROMReadInt(adressMQTT+8);
        Serial.println(myPort);

        if(myPort==65535)
    {
      myPort=0;
    }
        else
        {
                myIntMqttPort=myPort;
        }

        //Restore IP String
        if(firstIP!=65535 && secondIP!=65535 && thirdIP!=65535 &&
fourthIP!=65535)
        {
                myStrMqttServer =
String(firstIP)+"."+String(secondIP)+"."+String(thirdIP)+"."+String(fourthIP);

                setMQTTServer((char*)myStrMqttServer.c_str());
                Serial.println(myStrMqttServer);
        }
}*/

/*String SlowControl::getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length();

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}*/

/*void SlowControl::EEPROMWriteInt(int address, int number)
{
  EEPROM.write(address, number >> 8);
  EEPROM.write(address + 1, number & 0xFF);
}*/

/*int SlowControl::EEPROMReadInt(int address)
{
  return (EEPROM.read(address) << 8) + EEPROM.read(address + 1);
}*/
