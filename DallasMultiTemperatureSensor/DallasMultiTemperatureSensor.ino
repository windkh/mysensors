// Example sketch showing how to send in OneWire temperature readings

#define MY_NODE_ID AUTO
#define MY_DEBUG_VERBOSE
#define MY_SPECIAL_DEBUG

// Enable debug prints to serial monitor
#define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_NRF24

#include <MySensors.h>
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>

#define ONE_WIRE_BUS 3 // Pin where dallas sensor is connected 
#define SENSOR_COUNT 10 // max sensors

int numSensors = 0; // detected sensors

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

float lastTemperature[SENSOR_COUNT];

static unsigned long INTERVAL = 3000;
unsigned long previousMillis = 0;


// Initialize temperature message
MyMessage msg(0, V_TEMP);

const int ADDRESS_LENGTH = 8;
static uint8_t ADDRESSES[SENSOR_COUNT][ADDRESS_LENGTH]
{
	{ 0x28, 0x10, 0xA5, 0xE4, 0x5, 0x0, 0x0, 0xD6 },
	{ 0x28, 0x90, 0x85, 0xE7, 0x5, 0x0, 0x0, 0x4C },
	{ 0x28, 0xBC, 0xFE, 0xE5, 0x5, 0x0, 0x0, 0x9D },
	{ 0x28, 0xA6, 0xA2, 0xE4, 0x5, 0x0, 0x0, 0x32 },
	{ 0x28, 0xD5, 0xE6, 0xE4, 0x5, 0x0, 0x0, 0x05 },
	{ 0x28, 0x4D, 0x3F, 0xE5, 0x5, 0x0, 0x0, 0x37 },
	{ 0x28, 0xFB, 0x8C, 0xE5, 0x5, 0x0, 0x0, 0x83 },
	{ 0x28, 0x6F, 0xEC, 0xE4, 0x5, 0x0, 0x0, 0x70 },
	{ 0x28, 0xE4, 0xEC, 0xE5, 0x5, 0x0, 0x0, 0xED },
	{ 0x28, 0xD3, 0x58, 0xE5, 0x5, 0x0, 0x0, 0xD5 },
};

void presentation()
{
	sensors.begin();
	
	numSensors = sensors.getDeviceCount();
	Serial.print("Found ");
	Serial.print(numSensors, DEC);
	Serial.println(" sensors");

	// read addresses
	uint8_t address[ADDRESS_LENGTH];
	for (int i = 0; i < numSensors; i++)
	{
		sensors.getAddress(address, i);
		Serial.print("Sensor ");
		Serial.print(i, DEC);

		Serial.print(" address: 0x");
		Serial.print(address[0], HEX);
		Serial.print(", 0x");
		Serial.print(address[1], HEX);
		Serial.print(", 0x");
		Serial.print(address[2], HEX);
		Serial.print(", 0x");
		Serial.print(address[3], HEX);
		Serial.print(", 0x");
		Serial.print(address[4], HEX);
		Serial.print(", 0x");
		Serial.print(address[5], HEX);
		Serial.print(", 0x");
		Serial.print(address[6], HEX);
		Serial.print(", 0x");
		Serial.print(address[7], HEX);
		Serial.println("");
	}

	sendSketchInfo("Temperature Sensor", "2.0");

	for (int i = 0; i < numSensors; i++)
	{
		present(i, S_TEMP);
	}
}



void loop()
{
	if (millis() - previousMillis > INTERVAL)
	{
		previousMillis = millis();

		for (int i = 0; i < numSensors; i++)
		{
			uint8_t* pAddress = ADDRESSES[i];
			sensors.requestTemperaturesByAddress(pAddress);

			// Fetch and round temperature to one decimal
			float temperature = static_cast<float>(static_cast<int>(sensors.getTempC(pAddress) * 10.)) / 10.;

			// Only send data if temperature has changed and no error
			if (lastTemperature[i] != temperature && temperature != -127.00)
			{
				// Send in the new temperature
				send(msg.setSensor(i).set(temperature, 1));
				lastTemperature[i] = temperature;
			}
		}
	}
}
