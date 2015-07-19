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
#include <Battery.h>

#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1

#define HUMIDITY_SENSOR_DIGITAL_PIN 3
unsigned long SLEEP_TIME = 10000; // Sleep time between reads (in milliseconds)


MySensor gw;
Battery battery;
DHT dht;
float lastTemp;
float lastHum;
boolean metric = true;
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);

void setup()
{
	battery.begin(1200000, 400000);

	gw.begin(NULL, AUTO, false);
	dht.setup(HUMIDITY_SENSOR_DIGITAL_PIN);

	// Send the Sketch Version Information to the Gateway
	gw.sendSketchInfo("Humidity", "1.1");

	// Register all sensors to gw (they will be created as child devices)
	gw.present(CHILD_ID_HUM, S_HUM);
	gw.present(CHILD_ID_TEMP, S_TEMP);

	metric = gw.getConfig().isMetric;
}

void loop()
{
	delay(dht.getMinimumSamplingPeriod());

	float temperature = dht.getTemperature();
	if (isnan(temperature))
	{
		Serial.println("Failed reading temperature from DHT");
	}
	else if (temperature != lastTemp)
	{
		lastTemp = temperature;
		if (!metric)
		{
			temperature = dht.toFahrenheit(temperature);
		}
		gw.send(msgTemp.set(temperature, 1));
		Serial.print("T: ");
		Serial.println(temperature);
	}

	float humidity = dht.getHumidity();
	if (isnan(humidity))
	{
		Serial.println("Failed reading humidity from DHT");
	}
	else if (humidity != lastHum)
	{
		lastHum = humidity;
		gw.send(msgHum.set(humidity, 1));
		Serial.print("H: ");
		Serial.println(humidity);
	}

	gw.sleep(SLEEP_TIME); //sleep a bit

	int batteryPercent;
	if (battery.checkVoltage(&batteryPercent))
	{
		gw.sendBatteryLevel(batteryPercent);
	}
}


