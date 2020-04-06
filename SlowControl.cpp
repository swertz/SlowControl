/***************************************************
  This is a library for Slow Control System 

  Designed specifically to work with the Slow Control Board from UCLouvain designed by Antoine Deblaere
  ----> lien github

  This library will allow you to easily connect to WIFI, communicate with Sensors, and more
  

  Written by Antoine Deblaere for IRMP/CP3
  BSD license, all text above must be included in any redistribution
 ****************************************************/
 
#include "SlowControl.h"

SlowControl::SlowControl()
{
    //Declare instance of OneWire
    OneWire oneWire(SLOWCONTROL_DEFAULT_ONEWIRE);
    
    //Setup DallasLibrary Reference
    ds.setOneWire(&oneWire);
    
    //Pass WiFi Connection parameters to MQTT
    mqttClient.setClient(espClient);
    
    //Setup Default Variables
    dsFound=false;
    shtFound=false;
    tempSHTC=0.0;
    humiSHT=0.0;
    deviceTempCount = 0;
    ttlSimulate=false;
    ttlStatus=false;
    myConnectToTTL=false;
	myMqttServer="";
}

void SlowControl::begin(const char* mqtt_server)
{
	//Get reference of server
	myMqttServer=mqtt_server;
	
    //Setup MQTT Server    
    mqttClient.setServer(mqtt_server,1883);
    
    //Setup Callback
    mqttClient.setCallback(([this] (char* topic, byte* payload, unsigned int length) { this->callbackTTL(topic, payload, length); }));
    
    //Start DS Sensors Communication
    ds.begin();
    
    //Start Humidity Sensors Communication
    sht.begin(SLOWCONTROL_DEFAULT_HUMIDITSENSOR_ADDRESS,SLOWCONTROL_DEFAULT_SDA_PIN,SLOWCONTROL_DEFAULT_SCL_PIN);
}

void SlowControl::run(bool statusReadTempDS,bool statusReadTempSHT, bool statusReadHumiditySHT, bool statusCalculateDewPointSHT, bool publishToMQTT)
{
    //Check MQTT Connection
    if(!mqttClient.connected())
    {
        connectToMQTT(1,myClientID,myConnectToTTL); // Try to reconnect to MQTT one time.
    }
    mqttClient.loop();
    
    //Check Need of Reading
    if(statusReadTempDS)
    {
        //Read DS18B20 Temperature
        readTemperature();
    }
    
    //Check Need of Reading
    if(statusReadTempSHT)
    {
        ///Read SHT85 Temperature
        readTempSHT();
    }
    
    //Check Need of Reading
    if(statusReadHumiditySHT)
    {
        //Read SHT85 Humidity
        readHumiditySHT();
    }
    
    //Check Need of Reading
    if(statusCalculateDewPointSHT)
    {
        //Calculate SHT85 Dewpoint
        calculateDewPointSHT();    
    }
    
    //Check Need of Publishing to MQTT
    if(publishToMQTT)
    {
        if(mqttClient.connected())
        {
            //Publish readings to MQTT Topics
            mqttClient.publish(temperatureDS_topic,String(tempDSC).c_str());
            mqttClient.publish(temperatureSHT_topic,String(tempSHTC).c_str());
            mqttClient.publish(humiditySHT_topic,String(humiSHT).c_str());
            mqttClient.publish(dewPointSHT_topic,String(dewPointSHT).c_str());
            if(ttlSimulate==false)
            {
                ttlSimulate=true;
                mqttClient.publish(strcat(statusTTL_topic,myClientID),String(ttlStatus).c_str());
            }
        }
        else
        {
            Serial.println("Not Connected to MQTT Server. Please be sure that your MQTT Server is functional.");
            Serial.println();
        }
    }
}

void SlowControl::connectToWifi()
{
    //Exit after config instead of connecting
    wifiManager.setBreakAfterConfig(true);
    
    if (!wifiManager.autoConnect("AutoConnectAP", "password")) 
    {
        Serial.println("failed to connect, we should reset as see if it connects");
        delay(3000);
        ESP.reset();
        delay(5000);
    }
    Serial.println("Connected to WIFI");
    
    Serial.print("The IP Address of the module is :");
    Serial.println(WiFi.localIP());
    Serial.println();
}

void SlowControl::resetWifiSettings()
{
    wifiManager.resetSettings();
}

void SlowControl::connectToMQTT(int nbr,const char* clientID,bool connectToTTL)
{
    myClientID=clientID;
    myConnectToTTL=connectToTTL;
    //Try to connect to MQTT Server 5 times | 5 S timeout
    while (!mqttClient.connected())
    {
        for (int i=0;i<nbr;i++)
        {
            Serial.println("Attempting MQTT Connection..."+String(i+1)+"/"+String(nbr)+ " targeting -> "+myMqttServer);
            if(mqttClient.connect(myClientID))
            {
                Serial.println("Connected to MQTT Server.");
                Serial.println();
                
                if(connectToTTL==true)
                {
                    subscribeToSCBStatus();
                    Serial.println("Subscribed to Slow Control Board Status");
                    Serial.println();
                }
                else
                {
                    Serial.println("You didn't subcrisbed to Slow Control Board Status");
                    Serial.println();
                }
                break;
            }
            else
            {
            delay(5000);
            }
        }
        Serial.println();
        break;
    }
}

void SlowControl::publishToMQTT(const char* topic, const char* payload) 
{
    mqttClient.publish(topic,payload);
}

void SlowControl::subscribeToSCBStatus(const char* topic) 
{
    mqttClient.subscribe(topic);
}

void SlowControl::callbackTTL(char* topic, byte* payload, unsigned int length)
{
    Serial.print("Message arrived");
    String myMsg;
    Serial.println(topic);
    if(String(topic) == statusTTL_topic)
    {
        myMsg = byteArrayToString(payload,length);
        Serial.println(myMsg);
    }
    else
    {

    }
}

void SlowControl::getNumberOfTemperatureSensors()
{
    // Locating devices on the bus
    Serial.print("Locating temperatures sensors...");
    Serial.print("Found ");
    deviceTempCount = ds.getDeviceCount();
    Serial.print(deviceTempCount, DEC);
    Serial.println(" devices.");
    Serial.println("");
	dsFound=true;
}

void SlowControl::getNumberOfHumiditySensors()
{
    //Try to communicate
    if(sht.readStatus()!=0xFFFF)
    {
        shtFound=true;
        Serial.println("Can communicate with Humidty Sensors");
        Serial.println();
    }
    else
    {
        Serial.println("I2C Communication could not be etablished. Check your wiring !");
        Serial.println();
    }
}

void SlowControl::readTemperature()
{
    if(dsFound==true)
    {
        // Send command to all the sensors for temperature conversion
        ds.requestTemperatures(); 

        // Display temperature from each sensor
        for (int i = 0;  i < deviceTempCount;  i++)
        {
            Serial.print("Sensor ");
            Serial.print(i+1);
            Serial.print(" : ");
            tempDSC = ds.getTempCByIndex(i);
            Serial.print(tempDSC);
            Serial.print((char)176);//shows degrees character
            Serial.print("C ");
        }    
    }
    else
    {
        
    }
}

void SlowControl::readTempSHT()
{
    tempSHTC=sht.readTemperature();
    
    if (! isnan(tempSHTC)) 
    {
        Serial.print("SHT-85 | Temp *C = "); Serial.println(tempSHTC);
    } 
    else 
    { 
        Serial.println("Failed to read temperature");
    }
}

void SlowControl::readHumiditySHT()
{
    humiSHT=sht.readHumidity();
    
    if (! isnan(humiSHT)) 
    {
        Serial.print("SHT-85 | Hum. % = "); Serial.println(humiSHT);
    } 
    else 
    { 
        Serial.println("Failed to read humidity");
    }
}

void SlowControl::calculateDewPointSHT()
{
    if(shtFound==true)
    {
        float tn = tempSHTC < 0.0 ? 272.62 : 243.12;
        float m = tempSHTC < 0.0 ? 22.46 : 17.62;

        float l = logf(humiSHT / 100.0);
        float r = m * tempSHTC / (tn + tempSHTC);
        
        dewPointSHT=tn * (l + r) / (m - l - r);

        Serial.println(dewPointSHT);
    }
    else
    {
        Serial.println("DewPoint couldn't be calculated. Please be sure that you're getting readings of the SHT Sensors.");
        Serial.println();
    }
    
}


//Private Fuctions

String SlowControl::byteArrayToString( byte*payload, unsigned int length)
{
  String myMsg;
  
  for (int i = 0; i < length; i++) 
    {
      myMsg+= (char)payload[i];
    }
  return myMsg;
}














