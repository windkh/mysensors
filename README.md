# mysensors
Arduino sketches based on mysensors.org library.

The sketches are part of my home automation and are derived from the original
samples provided by the mysensors project. Some sketches are merged samples to
have more than one sensor on one arduino.

Some sketches need additional libraries that you will have to download for yourselve.


MySensors 2.0
GatewayW5100 - a standard Arduino Uno with Ethernet Shield gateway (see mysensors.org)
ClearEepromConfig - a modified copy of the original one from mysensors.org
CO2Sensor - a repeater-sketch that makes use of several sensors: MQ135, DHT-22, BMP085
FlamingoSwitchSensor - a repeater-sketch to control 433MHz switches from Flamingo/ELRO (with DHT-22)
ValloxSensor - a repeater-sketch for controlling the Vallox Digit-SE via RS485


MySensors 1.3 (not ported so far)
DallasMultiTemperatureSensor - a repeater-sketch for 10 DS1830 Temperature sensors
HumiditySensor - a DHT-22 sketch
WeatherStationSensor - a repeater-sketch (weatherstation) with BMP085 and DHT-22
