#include <SlowControl.h>

SlowControl slowControl;

void setup()
{
	//Initialize Serial Communication
	Serial.begin(115200);

	//Start Slow Control Library
	slowControl.begin();

	//Reset WiFi Settings if needed
	//slowControl.resetWifiSettings();

	//Initialize WiFi Communication
	slowControl.connectToWifi();

	//Initialize MQTT Communication
	slowControl.connectToMQTT(3,"ESP01",true); //Parameter 1 = Number of Try  || Parameter 2 = ID of the Board || Parameter 3 = Subscription to TTl Status 

	//Get Number Of Temperature Sensors on the OneWire Bus
	slowControl.getNumberOfTemperatureSensors();

	//Get Number Of Humidity Sensors on the I2C Bus
	slowControl.getNumberOfHumiditySensors();
}

void loop()
{
	//Parameter 1 = Read DS Temp  || Parameter 2 = Read SHT Temp
	//Parameter 3 = Read SHT Humi || Parameter 4 = Calculate DewPoint 
	//Parameter 5 = Publish Values to MQTT
	slowControl.run(true,true,true,true,true);
	
	delay(5000);
}
