/* (c) windkh 2017
MySensors 2.0
Bosch BME680 Sensor
required hardware:
- 1x Arduino
- 1x Radio NRF24L01+
- 1x BME680 (https://github.com/watterott/BME680-Breakout)

required libraries:
  https://github.com/windkh/BME680_Breakout

Notes on AirQuality output:
IAQ index value is between 0-500
  0-50  good
 51-100 average
101-150 little bad
151-200 bad
201-300 worse
301-500 very bad
--> TODO: right now this IAQ index is missing in the code
*/

#define MY_DEBUG_VERBOSE
#define MY_SPECIAL_DEBUG

// Enable debug prints to serial monitor
#define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_NRF24

#include <MySensors.h>
#include <Wire.h>
#include <SPI.h>

#include <BME680_Library.h>

BME680_Library bme680;

bool sendAlways = true; // set this to false if you only want to send values when they are changed.

const float SEALEVEL = 688; // meters above sealevel. We assume that this sensor is located at some static place
const float SEALEVEL_PRESSURE = 1013.25;
unsigned long SLEEP_TIME = 2000; // ms


// ----------------------------------------------------------------------------
#define CHILD_ID_PRESSURE 0
#define CHILD_ID_HUM 1
#define CHILD_ID_TEMP 2
#define CHILD_ID_RESISTANCE 3

boolean metric = true;

float lastHum = -1;
float lastPressure = -1;
float lastTemp = -1;
float lastResistance = -1;

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage pressureMsg(CHILD_ID_PRESSURE, V_PRESSURE);
MyMessage resistanceMsg(CHILD_ID_RESISTANCE, V_VAR1);


void presentation()
{
	sendSketchInfo("BME680 Sensor", "2.0");

	initSensor();

	present(CHILD_ID_PRESSURE, S_BARO);
	present(CHILD_ID_HUM, S_HUM);
	present(CHILD_ID_TEMP, S_TEMP);
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
	Wire.begin();

	if (!bme680.begin())
	{
		Serial.println("Initialization of BME680 failed!");
		for (;;); // spin forever
	}
	
	if (!bme680.configureForcedMode())
	{
		Serial.println("Configuring force mode of BME680 failed!");
		for (;;); // spin forever
	}
}


// derived from the example code which can be found here
// https://github.com/vicatcu/BME680_Breakout/tree/master/BME680_Library/examples/Basic
void updateSensor(void)
{
	bme680.configureForcedMode();
	if (bme680.read())
	{
		// Temperature ----------------------------------------------------------------------------------------------------
		float temp = bme680.getTemperature();
		if (lastTemp != temp || sendAlways)
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
		float humidity = bme680.getRelativeHumidity();
		if (lastHum != humidity || sendAlways)
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
		float absolutePressure = bme680.getBarometricPressure();
		float pressure = getSeaLevelPressure(SEALEVEL, absolutePressure);
		if (lastPressure != pressure || sendAlways)
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


		// Gas Resistance -----------------------------------------------------------------------------------------------------
		float gasResistance = bme680.getGasResistance();
		if (lastResistance != gasResistance || sendAlways)
		{
			lastResistance = gasResistance;
			Serial.print(F("R = "));
			Serial.print(lastResistance);
			Serial.println(F("Ohms"));
			if (!send(resistanceMsg.set(lastResistance, 1)))
			{
				lastResistance = -1.0;
			}
		}
	}
	else
	{
		Serial.println("Reading from BME680 failed!");
	}
}


// Returns the normalized pressure at sealevel: derived from BMP datasheet
float getSeaLevelPressure(float altitude, float absolutePressure)
{
	return absolutePressure / pow(1.0 - (altitude / 44330.0), 5.255);
}