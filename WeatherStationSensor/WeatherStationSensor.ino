/* (c) windkh 2016
MySensors 2.0
WeatherStation Sensor with DHT22 and BMP180
required hardware:
- 1x Arduino
- 1x Radio NRF24L01+
- 1x BMP180 or BMP085
- 1x DHT11

required software
https://github.com/adafruit/Adafruit_BMP085_Unified
https://github.com/adafruit/Adafruit_Sensor
http://www.github.com/markruys/arduino-DHT
*/

#define MY_NODE_ID AUTO
#define MY_DEBUG_VERBOSE
#define MY_SPECIAL_DEBUG

// Enable debug prints to serial monitor
#define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_NRF24

#include <MySensors.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

#include <DHT.h>  


const bool sendAlways = true;

const float SEALEVEL = 688; // Westendorf
const float SEALEVEL_PRESSURE = 1013.25;

// ----------------------------------------------------------------------------
#define CHILD_ID_HUM 2
#define CHILD_ID_TEMP 3
#define HUMIDITY_SENSOR_DIGITAL_PIN 3

DHT dht;
float lastTemp;
float lastHum;
boolean metric = true;
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);


// ----------------------------------------------------------------------------
#define BARO_CHILD 0
#define TEMP_CHILD 1

unsigned long SLEEP_TIME1 = 30000; // 1 minute required for forecast algorithm
unsigned long SLEEP_TIME2 = 30000; // 1 minute required for forecast algorithm

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(1);    // Digital Pressure Sensor 

/*
DP/Dt explanation
0 = "Stable Weather Pattern"
1 = "Slowly rising Good Weather", "Clear/Sunny "
2 = "Slowly falling L-Pressure ", "Cloudy/Rain "
3 = "Quickly rising H-Press",     "Not Stable"
4 = "Quickly falling L-Press",    "Thunderstorm"
5 = "Unknown (More Time needed)
*/

const char *weatherStrings[] = { "stable", "sunny", "cloudy", "unstable", "thunderstorm", "unknown" };
enum FORECAST
{
	STABLE = 0,			// Stable weather
	SUNNY = 1,			// Slowly rising HP stable good weather
	CLOUDY = 2,			// Slowly falling Low Pressure System, stable rainy weather
	UNSTABLE = 3,		// Quickly rising HP, not stable weather
	THUNDERSTORM = 4,	// Quickly falling LP, Thunderstorm, not stable
	UNKNOWN = 5			// Unknown, more time needed
};

const char *situationStrings[] = { "very low", "low", "normal", "high", "very high" };
enum WEATHER_SITUATION
{
	VERY_LOW_PRESSURE = 0,		// Tiefdruck p>-7.5hPa
	LOW_PRESSURE = 1,			// Tiefdruck p>-2.5hPa
	NORMAL_PRESSURE = 2,		// Normal	 p <+/-02.5hPa	
	HIGH_PRESSURE = 3,			// Hochdruck p>2.5hPa
	VERY_HIGH_PRESSURE = 4,		// Hochdruck p>7.5hPa
};

float lastPressure = -1;
float lastPressureTemp = -1;
int lastForecast = -1;
int lastSituation = NORMAL_PRESSURE;

const int LAST_SAMPLES_COUNT = 5;
float lastPressureSamples[LAST_SAMPLES_COUNT];

// get kPa/h be dividing hPa by 10 
#define CONVERSION_FACTOR (1.0/10.0)

int minuteCount = 0;
bool firstRound = true;

// average value is used in forecast algorithm.
float pressureAvg;
// average after 2 hours is used as reference value for the next iteration.
float pressureAvg2;
float dP_dt;

MyMessage tempMsg(TEMP_CHILD, V_TEMP);
MyMessage pressureMsg(BARO_CHILD, V_PRESSURE);
MyMessage forecastMsg(BARO_CHILD, V_FORECAST);
MyMessage situationMsg(BARO_CHILD, V_VAR1);
MyMessage forecastMsg2(BARO_CHILD, V_VAR2);

void displaySensorDetails(void)
{
	sensor_t sensor;
	bmp.getSensor(&sensor);
	Serial.println(F("------------------------------------"));
	Serial.print(F("Sensor:       ")); Serial.println(sensor.name);
	Serial.print(F("Driver Ver:   ")); Serial.println(sensor.version);
	Serial.print(F("Unique ID:    ")); Serial.println(sensor.sensor_id);
	Serial.print(F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" hPa"));
	Serial.print(F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" hPa"));
	Serial.print(F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" hPa"));
	Serial.println(F("------------------------------------"));
	Serial.println(F(""));
	delay(500);
}

void initPressureSensor()
{
	if (!bmp.begin())
	{
		Serial.println(F("Could not find a valid BMP085 sensor, check wiring!"));
		while (1) {}
	}

	displaySensorDetails();
}

void initHumiditySensor()
{
	dht.setup(HUMIDITY_SENSOR_DIGITAL_PIN);
}

void presentation()
{
	sendSketchInfo("Weather DHT11/BMP180", "2.3");

	initPressureSensor();
	initHumiditySensor();

	present(BARO_CHILD, S_BARO);
	present(TEMP_CHILD, S_TEMP);
	present(CHILD_ID_HUM, S_HUM);
	present(CHILD_ID_TEMP, S_TEMP);

	metric = getConfig().isMetric;
}

int getWeatherSituation(float pressure)
{
	int situation = NORMAL_PRESSURE;

	float delta = pressure - SEALEVEL_PRESSURE;
	if (delta > 7.5)
	{
		situation = VERY_HIGH_PRESSURE;
	}
	else if (delta > 2.5)
	{
		situation = HIGH_PRESSURE;
	}
	else if (delta < -7.5)
	{
		situation = VERY_LOW_PRESSURE;
	}
	else if (delta < -2.5)
	{
		situation = LOW_PRESSURE;
	}
	else
	{
		situation = NORMAL_PRESSURE;
	}

	return situation;
}

bool updatePressureSensor()
{
	bool changed = false;

	sensors_event_t event;
	bmp.getEvent(&event);

	if (event.pressure)
	{
		float absolutePressure = event.pressure;
		Serial.print(F("abs Pressure = "));
		Serial.print(absolutePressure);
		Serial.println(F(" hPa"));

		//float altitude = bmp.pressureToAltitude(SEALEVEL_PRESSURE, pressure);
		//Serial.print(F("Altitude = "));
		//Serial.print(altitude);
		//Serial.println(F(" m"));

		//sealevel pressure p0 from absolute pressure.
		float pressure = bmp.seaLevelForAltitude(SEALEVEL, absolutePressure);

		float pressureTemperature;
		bmp.getTemperature(&pressureTemperature);

		if (!metric)
		{
			// Convert to fahrenheit
			pressureTemperature = pressureTemperature * 9.0 / 5.0 + 32.0;
		}

		int forecast = sample(pressure);
		int situation = getWeatherSituation(pressure);


		if (sendAlways || (pressureTemperature != lastPressureTemp))
		{
			changed = true;
			lastPressureTemp = pressureTemperature;
			Serial.print(F("Temperature = "));
			Serial.print(pressureTemperature);
			Serial.println(metric ? F(" *C") : F(" *F"));
			if (!send(tempMsg.set(lastPressureTemp, 1)))
			{
				lastPressureTemp = -1.0;
			}
		}


		if (sendAlways || (pressure != lastPressure))
		{
			changed = true;
			lastPressure = pressure;
			Serial.print(F("sealevel Pressure = "));
			Serial.print(pressure);
			Serial.println(F(" hPa"));
			if (!send(pressureMsg.set(lastPressure, 1)))
			{
				lastPressure = -1.0;
			}
		}


		if (sendAlways || (forecast != lastForecast))
		{
			changed = true;
			lastForecast = forecast;
			Serial.print(F("Forecast = "));
			Serial.println(weatherStrings[forecast]);
			if (send(forecastMsg.set(weatherStrings[lastForecast])))
			{
				if (!send(forecastMsg2.set(lastForecast)))
				{
				}
			}
			else
			{
				lastForecast = -1.0;
			}
		}


		if (sendAlways || (situation != lastSituation))
		{
			changed = true;
			lastSituation = situation;
			Serial.print(F("Situation = "));
			Serial.println(situationStrings[situation]);
			if (!send(situationMsg.set(lastSituation, 0)))
			{
				lastSituation = -1.0;
			}
		}
	}

	return changed;
}

bool updateHumiditySensor()
{
	bool changed = false;

	float temperature = dht.getTemperature();
	float humidity = dht.getHumidity();

	if (!isnan(temperature))
	{
#ifdef SEND_WHEN_CHANGED
		if (temperature != lastTemp)
#endif
		{
			lastTemp = temperature;
			if (!metric)
			{
				temperature = dht.toFahrenheit(temperature);
			}
			changed = true;
			Serial.print(F("T: "));
			Serial.println(temperature);
			if (!send(msgTemp.set(temperature, 1)))
			{
				lastTemp = -1.0;
			}
		}
	}
	else
	{
		Serial.println(F("Failed reading temperature from DHT"));
	}


	if (!isnan(humidity))
	{
#ifdef SEND_WHEN_CHANGED
		if (humidity != lastHum)
#endif
		{
			lastHum = humidity;
			changed = true;
			Serial.print(F("H: "));
			Serial.println(humidity);
			if (!send(msgHum.set(lastHum, 1)))
			{
				lastHum = -1.0;
			}
		}
	}
	else
	{
		Serial.println(F("Failed reading humidity from DHT"));
	}

	return changed;
}

void loop()
{
	updatePressureSensor();
	sleep(SLEEP_TIME1);
	updateHumiditySensor();
	sleep(SLEEP_TIME2);
}


float getLastPressureSamplesAverage()
{
	float lastPressureSamplesAverage = 0;
	for (int i = 0; i < LAST_SAMPLES_COUNT; i++)
	{
		lastPressureSamplesAverage += lastPressureSamples[i];
	}
	lastPressureSamplesAverage /= LAST_SAMPLES_COUNT;

	//Serial.print(F("### 5min-Average:"));
	//Serial.print(lastPressureSamplesAverage);
	//Serial.println(F(" hPa"));

	return lastPressureSamplesAverage;
}



// Algorithm found here
// http://www.freescale.com/files/sensors/doc/app_note/AN3914.pdf
// Pressure in hPa -->  forecast done by calculating kPa/h
int sample(float pressure)
{
	// Calculate the average of the last n minutes.
	int index = minuteCount % LAST_SAMPLES_COUNT;
	lastPressureSamples[index] = pressure;

	minuteCount++;
	if (minuteCount > 185)
	{
		minuteCount = 6;
	}

	if (minuteCount == 5)
	{
		pressureAvg = getLastPressureSamplesAverage();
	}
	else if (minuteCount == 35)
	{
		float lastPressureAvg = getLastPressureSamplesAverage();
		float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
		if (firstRound) // first time initial 3 hour
		{
			dP_dt = change * 2; // note this is for t = 0.5hour
		}
		else
		{
			dP_dt = change / 1.5; // divide by 1.5 as this is the difference in time from 0 value.
		}
	}
	else if (minuteCount == 65)
	{
		float lastPressureAvg = getLastPressureSamplesAverage();
		float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
		if (firstRound) //first time initial 3 hour
		{
			dP_dt = change; //note this is for t = 1 hour
		}
		else
		{
			dP_dt = change / 2; //divide by 2 as this is the difference in time from 0 value
		}
	}
	else if (minuteCount == 95)
	{
		float lastPressureAvg = getLastPressureSamplesAverage();
		float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
		if (firstRound) // first time initial 3 hour
		{
			dP_dt = change / 1.5; // note this is for t = 1.5 hour
		}
		else
		{
			dP_dt = change / 2.5; // divide by 2.5 as this is the difference in time from 0 value
		}
	}
	else if (minuteCount == 125)
	{
		float lastPressureAvg = getLastPressureSamplesAverage();
		pressureAvg2 = lastPressureAvg; // store for later use.
		float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
		if (firstRound) // first time initial 3 hour
		{
			dP_dt = change / 2; // note this is for t = 2 hour
		}
		else
		{
			dP_dt = change / 3; // divide by 3 as this is the difference in time from 0 value
		}
	}
	else if (minuteCount == 155)
	{
		float lastPressureAvg = getLastPressureSamplesAverage();
		float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
		if (firstRound) // first time initial 3 hour
		{
			dP_dt = change / 2.5; // note this is for t = 2.5 hour
		}
		else
		{
			dP_dt = change / 3.5; // divide by 3.5 as this is the difference in time from 0 value
		}
	}
	else if (minuteCount == 185)
	{
		float lastPressureAvg = getLastPressureSamplesAverage();
		float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
		if (firstRound) // first time initial 3 hour
		{
			dP_dt = change / 3; // note this is for t = 3 hour
		}
		else
		{
			dP_dt = change / 4; // divide by 4 as this is the difference in time from 0 value
		}
		pressureAvg = pressureAvg2; // Equating the pressure at 0 to the pressure at 2 hour after 3 hours have past.
		firstRound = false; // flag to let you know that this is on the past 3 hour mark. Initialized to 0 outside main loop.
	}

	int forecast = UNKNOWN;
	if (minuteCount < 35 && firstRound) //if time is less than 35 min on the first 3 hour interval.
	{
		forecast = UNKNOWN;
	}
	else if (dP_dt < (-0.25))
	{
		forecast = THUNDERSTORM;
	}
	else if (dP_dt > 0.25)
	{
		forecast = UNSTABLE;
	}
	else if ((dP_dt > (-0.25)) && (dP_dt < (-0.05)))
	{
		forecast = CLOUDY;
	}
	else if ((dP_dt > 0.05) && (dP_dt < 0.25))
	{
		forecast = SUNNY;
	}
	else if ((dP_dt >(-0.05)) && (dP_dt < 0.05))
	{
		forecast = STABLE;
	}
	else
	{
		forecast = UNKNOWN;
	}

	Serial.print(F("Forecast at minute "));
	Serial.print(minuteCount);
	Serial.print(F(" dP/dt = "));
	Serial.print(dP_dt);
	Serial.print(F("kPa/h --> "));
	Serial.println(weatherStrings[forecast]);

	return forecast;
}