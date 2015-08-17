/*
Arduino Uno
PIN 2   grau           	NRF IRQ
PIN 3   grün           	DHT22 PIN 2
PIN 9   orange			NRF CE
PIN 10  gelb			NRF CSN
PIN 11  blau			NRF MOSI
PIN 12  violett			NRF MISO
PIN 13  grün			NRF SCK

GND 	schwarz			DHT22 PIN4
GND     braun           NRF GND
3.3V 	rot				DHT22 PIN1
3.3V    rot             NRF VCC

DHT22 PIN3 unbelegt
Widerstand 10K DHT22 PIN2 --> 3.3V

*/

#include <SPI.h>
#include <MySensor.h>  
#include <DHT.h>  


#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1

#define HUMIDITY_SENSOR_DIGITAL_PIN 3
unsigned long SLEEP_TIME = 10000; // Sleep time between reads (in milliseconds)

// #define BATTERY_POWERED
#ifdef BATTERY_POWERED
#include <Battery.h>
Battery battery;
#endif


MySensor gw;
DHT dht;

float lastTemp;
float lastHum;
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);

void setup()
{
#ifdef BATTERY_POWERED
	battery.begin(1200000, 400000);
#endif

	gw.begin(NULL, AUTO, false);
	dht.setup(HUMIDITY_SENSOR_DIGITAL_PIN, DHT::AM2302);
	
	// Send the Sketch Version Information to the Gateway
	gw.sendSketchInfo("Humidity", "1.2");

	// Register all sensors to gw (they will be created as child devices)
	gw.present(CHILD_ID_HUM, S_HUM);
	gw.present(CHILD_ID_TEMP, S_TEMP);
}

bool temperatureChanged()
{
	bool changed = false;
	float temperature = dht.getTemperature();
	if (!isnan(temperature))
	{
		if (temperature != lastTemp)
		{
			lastTemp = temperature;
			changed = true;
		}

		Serial.print("T: ");
		Serial.println(temperature);
	}
	else
	{
		Serial.println("Failed reading temperature from DHT");
	}

	return changed;
}

bool humidityChanged()
{
	bool changed = false;

	float humidity = dht.getHumidity();
	if (!isnan(humidity))
	{
		if (humidity != lastHum)
		{
			lastHum = humidity;
			changed = true;
		}

		Serial.print("H: ");
		Serial.println(humidity);
	}
	else
	{
		Serial.println("Failed reading humidity from DHT");
	}

	return changed;
}

void loop()
{
	delay(dht.getMinimumSamplingPeriod());
	
	bool tempChanged = temperatureChanged();
	bool humChanged = humidityChanged();
	if (tempChanged || humChanged)
	{
		gw.send(msgTemp.set(lastTemp, 1));
		gw.send(msgHum.set(lastHum, 1));
	}

	gw.sleep(SLEEP_TIME); //sleep a bit
	
#ifdef BATTERY_POWERED
	int batteryPercent;
	if (battery.checkVoltage(&batteryPercent))
	{
		gw.sendBatteryLevel(batteryPercent);
	}
#endif
}


