/***************************************************
  This is a library for Slow Control System 

  Designed specifically to work with the Slow Control Board from UCLouvain designed by Antoine Deblaere
  ----> lien github

  This library will allow you to easily connect to WIFI, communicate with Sensors, and more
  

  Written by Antoine Deblaere for IRMP/CP3
  BSD license, all text above must be included in any redistribution
 ****************************************************/
 
#include "SlowControl.h"

#define my_mqtt_server       "xxx.cloudmqtt.com"
#define my_mqtt_port         "1883"

bool SlowControl::shouldSaveConfig=false;

SlowControl::SlowControl()
{
    //Pass WiFi Connection parameters to MQTT
    mqttClient.setClient(espClient);
    
    //Setup Default Variables
    ttlSimulate=false;
    ttlStatus=false;
    myConnectToTTL=false;
	/*myMqttServer=NULL;	
	myMqttPort=NULL;*/
	mqttServerSet=false;
	//shouldSaveConfig=false;
}

void SlowControl::begin()
{
	//Serial Begin
	Serial.begin(115200);
	
	//Set TTl as OUTPUT
	pinMode(SLOWCONTROL_DEFAULT_TTL,OUTPUT);
	digitalWrite(SLOWCONTROL_DEFAULT_TTL,LOW);
}

void SlowControl::run()
{
	//Check Serial Input
	receiveWithEndMarker();
	showNewData();
	
    //Check MQTT Connection
    if(!mqttClient.connected())
    {
        connectToMQTT(1,myClientID,myConnectToTTL); // Try to reconnect to MQTT one time.
    }
    mqttClient.loop();
    
    //Check Need of Publishing to MQTT
    /*if(publishToMQTT)
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
    }*/
}

void SlowControl::setMQTTServer(String mqtt_server,uint16_t mqtt_port)
{
	if(mqtt_server!="")
	{
	  //Get reference of server
	  mqtt_server.toCharArray(myMqttServer,15);
	
      //Setup MQTT Server    
      mqttClient.setServer(myMqttServer,mqtt_port);
    
      //Setup Callback
      mqttClient.setCallback(([this] (char* topic, byte* payload, unsigned int length) { this->callbackTTL(topic, payload, length); }));
	  
	  //Status of set
	  mqttServerSet=true;
	  
	}
	else
	{
		Serial.println("MQTT Serverd not hardcoded");
	}
}

void SlowControl::checkJSONConfig()
{
	Serial.println("mounting FS...");
	
	if(SPIFFS.begin())
	{
		Serial.println("Mounted File System");
		if(SPIFFS.exists("/config.json"))
		{
			Serial.println("Reading Config File");
			File configFile = SPIFFS.open("/config.json","r");
			if(configFile)
			{
				Serial.println("Opened Config File");
				size_t size = configFile.size();
				//Allocate a buffer to store the contents of the file.
				std::unique_ptr<char[]> buf(new char [size]);
				
				configFile.readBytes(buf.get(),size);
				DynamicJsonBuffer jsonBuffer;
				JsonObject& json = jsonBuffer.parseObject(buf.get());
				json.printTo(Serial);
				if (json.success())
				{
					strcpy(myMqttServer, json["mqtt_server"]);
					strcpy(myMqttPort, json["mqtt_port"]);
				}
				else
				{
					Serial.println("Failed to lad Json Config");
				}
			}
		}
		else
		{
			Serial.println("JSON File doesn't exist");
		}
			
	}
	else
	{
		Serial.println("Failed to mount FS");
	}
}
void SlowControl::saveConfigCallback()
{
	Serial.println("Should Save Config");
	shouldSaveConfig=true;
}

void SlowControl::connectToWifi()
{
	//Check Json Config File
	checkJSONConfig();
	
	//Set Config Save Notify Callback
	wifiManager.setSaveConfigCallback(&SlowControl::saveConfigCallback);
	
	WiFiManagerParameter custom_mqtt_server("server", "mqtt server", myMqttServer, 40);
	WiFiManagerParameter custom_mqtt_port("port", "mqtt port", myMqttPort, 6);
	
	//Check If MQTT Credentials have been hardcoded
	if(mqttServerSet==false)
	{   
      //wifiManager.setBreakAfterConfig(true); //Try to connect one time and thens exits.
	  wifiManager.addParameter(&custom_mqtt_server);
	  wifiManager.addParameter(&custom_mqtt_port);
	}
    
	//Start Acces Point Configuration if you never connected the board to a WiFi previously
    if (!wifiManager.autoConnect("AutoConnectAP", "password")) 
    {
        Serial.println("failed to connect, we should reset as see if it connects");
        delay(3000);
        ESP.reset();
        delay(5000);
    }
		
    Serial.println("Connected to WIFI");
	delay(1000);
	
	//Print MAC ADRESS
	Serial.println(WiFi.macAddress());
	
	//Get value
	strcpy(myMqttServer,custom_mqtt_server.getValue());
	strcpy(myMqttPort,custom_mqtt_port.getValue());
	
	//Save to FS
	if(shouldSaveConfig)
	{
		Serial.println("Saving Config");
		DynamicJsonBuffer jsonBuffer;
		JsonObject& json = jsonBuffer.createObject();
		json["mqtt_server"] = myMqttServer ;
		json["mqtt_port"] = myMqttPort ;
		
		File configFile = SPIFFS.open("/config.json","w");
		if(!configFile)
		{
			Serial.println("Failed to open config file for writing");
		}
		
		json.prettyPrintTo(Serial);
		json.printTo(configFile);
		configFile.close();
		shouldSaveConfig = false;
	}
}

void SlowControl::resetWifiSettings()
{
	mqttServerSet=false;
    wifiManager.resetSettings();
}

void SlowControl::connectToMQTT(int nbr,const char* clientID,bool connectToTTL)
{
	if(myMqttServer!="")
	{
		setMQTTServer(myMqttServer,String(myMqttPort).toInt());
	}
    myClientID=clientID;
    myConnectToTTL=connectToTTL;
    //Try to connect to MQTT Server 5 times | 5 S timeout
    while (!mqttClient.connected())
    {
        for (int i=0;i<nbr;i++)
        {
			Serial.println("Attempting MQTT Connection..."+String(i+1)+"/\0"+String(nbr)+ " targeting -> "+String(myMqttServer));

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

bool SlowControl::isConnected()
{
	if (mqttClient.connected())
	{
		return true;
	}
	else
	{
		return false;
	}
}

void SlowControl::publishToMQTT(const char* topic, const char* payload) 
{
    mqttClient.publish(topic,payload);
}

void SlowControl::publishValues(String *data_array)
{
	char buffer [100 ];

	for (int i=0;i<data_array->length();i++)
	{
		strcpy(buffer,myClientID);
		strcat(buffer,mqtt_topics[i].c_str());

		publishToMQTT(buffer,data_array[i].c_str());
	}
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

void SlowControl::set_TTL_OUTPUT(int state)
{
	digitalWrite(SLOWCONTROL_DEFAULT_TTL,state);
}

/*void SlowControl::getNumberOfTemperatureSensors()
{
    // Locating devices on the bus
    Serial.print("Locating temperatures sensors...");
    Serial.print("Found ");
    deviceTempCount = ds.getDeviceCount();
    Serial.print(deviceTempCount, DEC);
    Serial.println(" devices.");
    Serial.println("");
	
	//Check if there're some devices
	if(deviceTempCount!=0)
	{
		dsFound=true;
	}
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
    
}*/


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

void SlowControl::receiveWithEndMarker()
{
	static byte ndx = 0;
    char endMarker = '\@';
    char rc;
   
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}

void SlowControl::showNewData()
{
	if(newData==true)
	{
		resetWifiSettings();
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
	if(firstIP!=65535 && secondIP!=65535 && thirdIP!=65535 && fourthIP!=65535)
	{
		myStrMqttServer = String(firstIP)+"."+String(secondIP)+"."+String(thirdIP)+"."+String(fourthIP);
		
		setMQTTServer((char*)myStrMqttServer.c_str());
		Serial.println(myStrMqttServer);
	}
}*/

String SlowControl::getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length();

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

/*void SlowControl::EEPROMWriteInt(int address, int number)
{
  EEPROM.write(address, number >> 8);
  EEPROM.write(address + 1, number & 0xFF);
}*/

/*int SlowControl::EEPROMReadInt(int address)
{
  return (EEPROM.read(address) << 8) + EEPROM.read(address + 1);
}*/











