/*
Arduino nano
CO2 MySensor

MQ135
DHT22

PIN 2   grau            NRF IRQ
PIN 3   gruen           DHT22 PIN 2
PIN 9   orange          NRF CE
PIN 10  gelb            NRF CSN
PIN 11  blau            NRF MOSI
PIN 12  violett         NRF MISO
PIN 13  gruen           NRF SCK

GND     schwarz         DHT22 PIN4
GND     braun           NRF GND
3.3V    rot             DHT22 PIN1
3.3V    rot             NRF VCC

DHT22 PIN3 unbelegt
Widerstand 10K DHT22 PIN3 --> 3.3V

MQ135
A0      weiss
D0      schwarz unbelegt
GND     braun
5V      rot
*/

#include <SPI.h>
#include <MySensor.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

//#include "RunningAverage.h"

#include <DHT.h>  
#include <MQ135.h>

#include "Timer.h"

const float SEALEVEL = 688; // Westendorf
const float SEALEVEL_PRESSURE = 1013.25;

//-----------------------------------------------------------------------------
// Timer
Timer timer;
#define TEMP_UPDATE_INTERVAL 30000

// The pressure forecast algorithm requires the samples to be taken every minute.
#define PRESSURE_UPDATE_INTERVAL 60000

enum WEATHER_SITUATION
{
    VERY_LOW_PRESSURE = 0,      // Tiefdruck p>-7.5hPa
    LOW_PRESSURE = 1,           // Tiefdruck p>-2.5hPa
    NORMAL_PRESSURE = 2,        // Normal    p <+/-02.5hPa  
    HIGH_PRESSURE = 3,          // Hochdruck p>2.5hPa
    VERY_HIGH_PRESSURE = 4,     // Hochdruck p>7.5hPa
};


//-----------------------------------------------------------------------------
// BMP 085
#define CHILD_ID_BARO 5
#define CHILD_ID_BARO_TEMP 6

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(2);

float lastPressure = -1;
float lastBaroTemp = -1;
int lastForecast = -1;
int lastSituation = NORMAL_PRESSURE;

/*
DP/Dt explanation
0 = "Stable Weather Pattern"
1 = "Slowly rising Good Weather", "Clear/Sunny "
2 = "Slowly falling L-Pressure ", "Cloudy/Rain "
3 = "Quickly rising H-Press",     "Not Stable"
4 = "Quickly falling L-Press",    "Thunderstorm"
5 = "Unknown (More Time needed)
*/

const char *weather[] = { "stable", "sunny", "cloudy", "unstable", "thunderstorm", "unknown" };
enum FORECAST
{
    STABLE = 0,         // Stable weather
    SUNNY = 1,          // Slowly rising HP stable good weather
    CLOUDY = 2,         // Slowly falling Low Pressure System, stable rainy weather
    UNSTABLE = 3,       // Quickly rising HP, not stable weather
    THUNDERSTORM = 4,   // Quickly falling LP, Thunderstorm, not stable
    UNKNOWN = 5         // Unknown, more time needed
};


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


MyMessage tempMsg(CHILD_ID_BARO_TEMP, V_TEMP);
MyMessage pressureMsg(CHILD_ID_BARO, V_PRESSURE);
MyMessage forecastMsg(CHILD_ID_BARO, V_FORECAST);
MyMessage situationMsg(CHILD_ID_BARO, V_VAR1);

//-----------------------------------------------------------------------------
// DHT22
#define CHILD_ID_TEMP 2
#define CHILD_ID_HUM 3
#define HUMIDITY_SENSOR_DIGITAL_PIN 3

DHT dht;
float lastTemp;
float lastHum;
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);

//-----------------------------------------------------------------------------
// MQ135
#define CHILD_ID_CO2_CORRECTED 0
#define CHILD_ID_CO2 1
#define CHILD_ID_R0 4
#define CO2_SENSOR_ANALOG_PIN 0
#define CO2_SENSOR_BUFFER_SIZE 3

/// Calibration resistance at atmospheric CO2 level
// Buero 21Grad Regen
#define RZERO 300.0
#define EEPROM_R0 0

MQ135 gasSensor = MQ135(CO2_SENSOR_ANALOG_PIN, RZERO);
int lastC02;
int lastCO2Corrected;
float lastR0;

//RunningAverage lastCO2Values(CO2_SENSOR_BUFFER_SIZE);

//-----------------------------------------------------------------------------
// MySensor
MySensor gw;
MyMessage msgCO2Corrected(CHILD_ID_CO2_CORRECTED, V_VAR1);
MyMessage msgCO2(CHILD_ID_CO2, V_VAR1);
MyMessage msgR0(CHILD_ID_R0, V_VAR1);

//-----------------------------------------------------------------------------
void setup()
{
    dht.setup(HUMIDITY_SENSOR_DIGITAL_PIN);

    if (bmp.begin()) {
        // OK
    }
    else
    {
        Serial.println("Could not find a valid BMP085 sensor, check wiring!");
        while (1) {}
    }

    gw.begin(incomingMessage, AUTO, true);
    gw.sendSketchInfo("CO2 Sensor MQ-135", "1.3");
    gw.present(CHILD_ID_CO2_CORRECTED, S_AIR_QUALITY);
    gw.present(CHILD_ID_CO2, S_AIR_QUALITY);
    gw.present(CHILD_ID_TEMP, S_TEMP);
    gw.present(CHILD_ID_HUM, S_HUM);
    gw.present(CHILD_ID_R0, S_CUSTOM);
    gw.present(CHILD_ID_BARO, S_BARO);
    gw.present(CHILD_ID_BARO_TEMP, S_TEMP);
    

    uint8_t R02 = gw.loadState(EEPROM_R0);

    // get R0 from EEPROM
    float R0 = R02 * 2;
    if (R0 > 1.0 && R0 < 400.0)
    {
        Serial.print(F("Setting R0 from EEPROM: "));
    }
    else
    {
        Serial.print(F("Setting default R0: "));
        R0 = RZERO;
    }

    Serial.print(R0);
    Serial.println(F(""));

    gasSensor.setR0(R0);
    //float ppm = gasSensor.getPPM();
    //lastCO2Values.fillValue(ppm, CO2_SENSOR_BUFFER_SIZE);

    //int tickEvent1 = 
    timer.every(TEMP_UPDATE_INTERVAL, timerHandler);
    //int tickeEvent2 = 
    timer.every(PRESSURE_UPDATE_INTERVAL, pressureTimerHandler);
}

bool DHT22Changed(bool waitMinimumSamplingPeriod = true)
{
    bool changed = false;

    if (waitMinimumSamplingPeriod)
    {
        delay(dht.getMinimumSamplingPeriod());
    }

    float temperature = dht.getTemperature();
    if (isnan(temperature))
    {
        Serial.println(F("Failed reading temperature from DHT"));
    }
    else if (temperature != lastTemp)
    {
        lastTemp = temperature;
        Serial.print("T: ");
        Serial.println(temperature);
        changed = true;
    }

    float humidity = dht.getHumidity();
    if (isnan(humidity))
    {
        Serial.println(F("Failed reading humidity from DHT"));
    }
    else if (humidity != lastHum)
    {
        lastHum = humidity;
        Serial.print(F("H: "));
        Serial.println(humidity);
        changed = true;
    }

    return changed;
}

bool MQ135Changed(float t, float h)
{
    bool changed = false;

    lastR0 = gasSensor.getRZero();
    Serial.print(F("R0: "));
    Serial.println(lastR0);

    {
        float ppm = gasSensor.getPPM();

        Serial.print(F("CO2 ppm: "));
        Serial.print(ppm);

        //lastCO2Values.addValue(ppm);
        //ppm = lastCO2Values.getAverage();

        //Serial.print(" average: ");
        //Serial.print(ppm);

        int roundedPpm = (int)ppm;
        Serial.print(F(" --> "));
        Serial.println(roundedPpm);

        if (roundedPpm != lastC02)
        {
            lastC02 = roundedPpm;
            changed = true;
        }
    }

    {
        float ppm = gasSensor.getCorrectedPPM(t, h);

        Serial.print(F("CO2 corrected ppm: "));
        Serial.print(ppm);
        
        int roundedPpm = (int)ppm;
        Serial.print(F(" --> "));
        Serial.println(roundedPpm);

        if (roundedPpm != lastCO2Corrected)
        {
            lastCO2Corrected = roundedPpm;
            changed = true;
        }
    }

    return changed;
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

bool BMP085Changed()
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
        
        float temperature;
        bmp.getTemperature(&temperature);

        int forecast = sample(pressure);
        lastSituation = getWeatherSituation(pressure);

        Serial.print(F("Temperature = "));
        Serial.print(temperature);
        Serial.println(F(" *C"));
        Serial.print(F("Pressure = "));
        Serial.print(pressure);
        Serial.println(F(" hPa"));
        Serial.print(F("Forecast = "));
        Serial.println(weather[forecast]);
        
        if (temperature != lastBaroTemp) 
        {
            lastBaroTemp = temperature;
            changed = true;
        }

        if (pressure != lastPressure) 
        {
            lastPressure = pressure;
            
            changed = true;
        }

        if (forecast != lastForecast) 
        {
            lastForecast = forecast;
            changed = true;
        }
    }

    return changed;
}


void timerHandler()
{
    bool humidityChanged = DHT22Changed(false);
    bool airQualityChanged = MQ135Changed(lastTemp, lastHum);
    
    if (humidityChanged || airQualityChanged)
    {
        gw.send(msgTemp.set(lastTemp, 1));
        gw.send(msgHum.set(lastHum, 1));
        gw.send(msgCO2Corrected.set(lastCO2Corrected));
        gw.send(msgCO2.set(lastC02));
        gw.send(msgR0.set(lastR0, 2));
    }
}

void pressureTimerHandler()
{
    bool pressureChanged = BMP085Changed();

    if (pressureChanged) 
    {
        gw.send(tempMsg.set(lastBaroTemp, 1));
        gw.send(pressureMsg.set(lastPressure, 1));
        gw.send(forecastMsg.set(weather[lastForecast]));
        gw.send(situationMsg.set(lastSituation, 0));
    }
}

void loop()
{
    gw.process();

    timer.update();
}

void incomingMessage(const MyMessage& message)
{
    Serial.println(F("Incoming Message:"));

    if (message.isAck())
    {
        Serial.println(F("This is an ack from gateway"));
    }

    uint8_t sensor = message.sensor;
    if (sensor == CHILD_ID_R0)
    {
        float R0 = message.getFloat();

        Serial.print(F("Incoming R0: "));
        Serial.print(R0);
        Serial.println(F(""));

        gw.saveState(EEPROM_R0, (uint8_t)(R0/2));
        gasSensor.setR0(R0);
        gw.send(msgR0.set(R0, 2));
    }
}

float getLastPressureSamplesAverage()
{
    float lastPressureSamplesAverage = 0;
    for (int i = 0; i < LAST_SAMPLES_COUNT; i++)
    {
        lastPressureSamplesAverage += lastPressureSamples[i];
    }
    lastPressureSamplesAverage /= LAST_SAMPLES_COUNT;

    Serial.print(F("### 5min-Average:"));
    Serial.print(lastPressureSamplesAverage);
    Serial.println(F(" hPa"));

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
    Serial.println(weather[forecast]);

    return forecast;
}


