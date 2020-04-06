/***************************************************
  This is a library for Slow Control System 

  Designed specifically to work with the Slow Control Board from UCLouvain designed by Antoine Deblaere
  ----> lien github

  This library will allow you to easily connect to WIFI, communicate with Sensors, and more
  

  Written by Antoine Deblaere for IRMP/CP3
  BSD license, all text above must be included in any redistribution
 ****************************************************/
 
#ifndef SlowControl_H
#define SlowControl_H

#include "Arduino.h"
#include <DallasTemperature.h>
#include "ESP8266WiFi.h"
#include <PubSubClient.h>
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <SHT85.h>


#define SLOWCONTROL_DEFAULT_MQTT_SERVER "DESKTOP-09R46K7.local"
#define SLOWCONTROL_DEFAULT_MQTT_CLIENT_ID "SlowControlBoard"
#define SLOWCONTROL_DEFAULT_NBR_OF_TRY 5
#define SLOWCONTROL_DEFAULT_HUMIDITSENSOR_ADDRESS 0X44
#define SLOWCONTROL_DEFAULT_SDA_PIN 13
#define SLOWCONTROL_DEFAULT_SCL_PIN 14
#define SLOWCONTROL_DEFAULT_TTL 16
#define SLOWCONTROL_DEFAULT_ONEWIRE 4

#define temperatureDS_topic "sensors/temperature/DSTemp"
#define temperatureSHT_topic "sensors/temperature/SHTTemp"
#define humiditySHT_topic "sensors/humidity/SHTHumi"
#define dewPointSHT_topic "sensors/humidity/SHTDewPoint"
#define statusTTL_topic "status/ttl/" //Will use the ID of the Board to retrieve where the status come from

class SlowControl {
    public :
        /**
         *  Constructor.
         */
        SlowControl();
        
        /**
         * Initialises the Slow Control Library
         */
        void begin(const char* mqtt_server = SLOWCONTROL_DEFAULT_MQTT_SERVER); //--> Verified
        
        /**
         * Start running of the Slow Control Library
         */
        void run(bool statusReadTempDS,bool statusReadTempSHT, bool statusReadHumiditySHT, bool statusCalculateDewPointSHT, bool publishToMQTT); //--> Verified	

        /**
         * Initialises the WIFI Communication
         *
         * Take previous credentials if the board has already connected to Wifi
        *
         *If not create an Wifi AP to enter the credentials of our personal Network
         */
        void connectToWifi(); //--> Verified
        
        /**
         * Reset WiFi Settings
         *
         * In order to change WiFi Credentials, you can call this function to perform a reset of them
         *
         */
        void resetWifiSettings(); //--> Verified
        
        /**
         * Initialises the MQTT Communication
         *
         *Take default MQTT Server, Client ID & TTL Subscription feedback
         *
         *If it doesnt suit, pass yours as paramaters
         */
        void connectToMQTT(int nbr=SLOWCONTROL_DEFAULT_NBR_OF_TRY, const char* clientID = SLOWCONTROL_DEFAULT_MQTT_CLIENT_ID,bool connectToTTL=false); //--> Verified
        
        /**
         * Publish to MQTT Topic
         *
         *Param 1 : Topic where you want to publish || Param 2 : Data you want to publish
         *
         */
        void publishToMQTT(const char* topic, const char* payload);	//--> Verified
        
        /**
         * Subscribe to TTL Status
         *
         *Take default TTL Topic as paramaters
        *
        *If it doesnt suit, pass yours as paramaters
        */
        void subscribeToSCBStatus(const char* topic=statusTTL_topic); //--> Verified
        
        /**
         * Gets number of Temperature Sensors.
        *
         * Print on the serial monitor value indicating the number of DS sensors that are on the OneWire BUS.
         */
        void getNumberOfTemperatureSensors(); //--> Verified
        
        /**
         * Gets number of Humidity Sensors.
         *
         * Print on the serial monitor value indicating the number of SHT sensors that are on the OneWire BUS.
         */
        void getNumberOfHumiditySensors(); //--> Verified
        
        /**
         * Gets a single temperature reading from the sensor.
         *
         * Write value to global variable indicating the temperature of DS Sensor.
         */
        void readTemperature();
        
        /**
         * Gets a single relative humidity reading from the sensor.
         *
         * Write value to global variable indicating the temperature of SHT Sensor.
         */
        void readTempSHT();
        
        /**
         * Gets a single relative humidity reading from the sensor.
         *
         * Write value to global variable indicating the Humidity of SHT Sensor.
         */
        void readHumiditySHT();
        
        /**
         * Calculate dewPoint from Temperature & Humidity.
         *
         * Calculate from SHTTemp & SHTHumi and write value to global variable indicating the DewPoint .
         */
        void calculateDewPointSHT();

         /**
         * Setup callback function 
         *
         * Get feeback from topic that changed if you subscribed to them
         */
        void callbackTTL(char* topic, byte* payload, unsigned int length); 
        
    private :
        /**
         *  Constructor.
         */
        WiFiManager wifiManager;
        WiFiClient espClient;
        PubSubClient mqttClient;
        DallasTemperature ds;
        SHT85 sht;
        
        /**
         *  Status of Sensors Variables.
         */
        bool dsFound;
        bool shtFound;
        int deviceTempCount;
        float tempDSC;
        float tempSHTC;
        float humiSHT;
        double dewPointSHT;
        const char* myClientID;
        bool ttlSimulate;
        bool ttlStatus;
        bool myConnectToTTL;
		const char* myMqttServer;
        
        /**
         *  Private Functions
         */
        String byteArrayToString( byte*payload, unsigned int length);

};

#endif