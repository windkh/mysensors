/* (c) windkh 2017
MySensors 2.0
Bosch BME680 Sensor
required hardware:
- 1x Arduino
- 1x Radio NRF24L01+
- 1x BME680 (https://github.com/watterott/BME680-Breakout)

required libraries:
  https://github.com/DFRobot/DFRobot_BME680
*/

//#define MY_DEBUG_VERBOSE
//#define MY_SPECIAL_DEBUG

// Enable debug prints to serial monitor
// #define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_NRF24

#include <MySensors.h>
#include <Wire.h>
#include <SPI.h>

#include "DFRobot_BME680_I2C.h"

const uint8_t bme_addr = 0x77;		//bme I2C method address
DFRobot_BME680_I2C bme(bme_addr);	//I2C method

const float SEALEVEL = 688; // meters abobve sealevel. We assume that this sensor is located on some static place
const float SEALEVEL_PRESSURE = 1013.25;
unsigned long SLEEP_TIME = 10000; // 10


// ----------------------------------------------------------------------------
#define CHILD_ID_PRESSURE 0
#define CHILD_ID_HUM 1
#define CHILD_ID_TEMP 2
#define CHILD_ID_AIRQUALITY 3
#define CHILD_ID_RESISTANCE 4

boolean metric = true;

float lastHum = -1;
float lastPressure = -1;
float lastTemp = -1;
float lastAirQuality = -1;
float lastResistance = -1;

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage pressureMsg(CHILD_ID_PRESSURE, V_PRESSURE);
MyMessage airQualityMsg(CHILD_ID_AIRQUALITY, V_VAR1);
MyMessage resistanceMsg(CHILD_ID_RESISTANCE, V_VAR1);


void presentation()
{
	sendSketchInfo("BME680 Sensor", "2.0");

	initSensor();

	present(CHILD_ID_PRESSURE, S_BARO);
	present(CHILD_ID_HUM, S_HUM);
	present(CHILD_ID_TEMP, S_TEMP);
	present(CHILD_ID_AIRQUALITY, S_AIR_QUALITY);
	present(CHILD_ID_RESISTANCE, S_CUSTOM);

	metric = getConfig().isMetric;
}


void loop()
{
	updateSensor();
	sleep(SLEEP_TIME);
}


void initSensor()
{
	if (bme.begin()) {
		Serial.println("init sucessful");
	}
	else {
		Serial.println("init faild");
		for (;;);
	}
	Serial.println();
}


// derived from the example code which can be found here
// https://github.com/DFRobot/DFRobot_BME680/tree/master/examples/DFRobot_BME680_I2C
void updateSensor(void)
{
	bme.startConvert();					  //to get a accurate values
	delay(1);
	Serial.println();

	// Temperature ----------------------------------------------------------------------------------------------------
	float temp = bme.readTempture();      //float tempture, unit degree Celsius, this is account to two decimal places
	if (lastTemp != temp)
	{
		lastTemp = temp;
		Serial.print(F("Temperature = "));
		Serial.print(lastTemp);
		Serial.println(metric ? F("*C") : F("*F"));
		if (!send(msgTemp.set(lastTemp, 1)))
		{
			lastTemp = -1.0;
		}
	}

	// Humidity -------------------------------------------------------------------------------------------------------
	float humidity = bme.readHumidity();  //float humidity, Unit relative humidity, this is account to two decimal places
	if (lastHum != humidity)
	{
		lastHum = humidity;
		Serial.print(F("Humidity = "));
		Serial.print(lastHum);
		Serial.println(F("%"));
		if (!send(msgHum.set(lastHum, 1)))
		{
			lastTemp = -1.0;
		}
	}

	// Pressure -------------------------------------------------------------------------------------------------------
	float absolutePressure = bme.readPressure();  //float pressure, Unit Unit MPa, this is account to two decimal places
	float pressure = getSeaLevelPressure(SEALEVEL, absolutePressure) / 100.0; // hPa
	if (lastPressure != pressure)
	{
		lastPressure = pressure;
		Serial.print(F("sealevel Pressure = "));
		Serial.print(lastPressure);
		Serial.println(F("hPa"));
		if (!send(pressureMsg.set(lastPressure, 1)))
		{
			lastPressure = -1.0;
		}
	}

	// float altitude = bme.readAltitude();  //float altitude, Unit metre, this is account to one decimal places
	// This is not the absolute Altitude, but one calculated using a global atmospheric formula.

	// AirQuality -----------------------------------------------------------------------------------------------------
	uint16_t gas = bme.readGas();         //uint16_t gas, Unit ppm, this is account to one decimal places
	if (lastAirQuality != gas)
	{
		lastAirQuality = gas;
		Serial.print(F("gas = "));
		Serial.print(lastAirQuality);
		Serial.println(F(" ppm"));
		if (!send(airQualityMsg.set(lastAirQuality, 1)))
		{
			lastAirQuality = -1.0;
		}
	}


	// Resistance -----------------------------------------------------------------------------------------------------
	float gasResistance = bme.readGasResistance();  //float resistance, Unit ohm, this is account to two decimal places
	if (lastResistance != gasResistance)
	{
		lastResistance = gasResistance;
		Serial.print(F("R = "));
		Serial.print(lastResistance);
		Serial.println(F(" Ohm"));
		if (!send(resistanceMsg.set(lastResistance, 1)))
		{
			lastResistance = -1.0;
		}
	}
}


// Returns the normalized pressure at sealevel: derived from BMP datasheet
float getSeaLevelPressure(float altitude, float absolutePressure)
{
	return absolutePressure / pow(1.0 - (altitude / 44330.0), 5.255);
}