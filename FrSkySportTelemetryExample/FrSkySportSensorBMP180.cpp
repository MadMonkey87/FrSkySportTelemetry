#include "FrSkySportSensorBMP180.h"
#include <SFE_BMP180.h>
#include <Wire.h>
SFE_BMP180 pressureSensor;
#define ALTITUDE 548.0 // base altitude

FrSkySportSensorBMP180::FrSkySportSensorBMP180(SensorId id) : FrSkySportSensor(id)
{
}

void FrSkySportSensorBMP180::setup()
{
  Serial.println("Initialize BMP180...");
  sensorInitialized = pressureSensor.begin();

  if (sensorInitialized)
  {

    char waitingTime = pressureSensor.startTemperature();
    Serial.print("waiting for temperature sensor (ms): ");
    Serial.println((uint16_t)waitingTime);
    delay(waitingTime);

    waitingTime = pressureSensor.startPressure(1);
    Serial.print("waiting for pressure sensor (ms): ");
    Serial.println((uint16_t)waitingTime);
    delay(waitingTime);

    Serial.println("...done!");
  }
  else
  {
    Serial.println("...failed!");
  }
}

uint16_t FrSkySportSensorBMP180::send(FrSkySportSingleWireSerial &serial, uint8_t id, uint32_t now)
{
  uint16_t dataId = SENSOR_NO_DATA_ID;
  if (sensorId == id)
  {
    switch (sensorDataIdx)
    {
    case 0:
      dataId = BMP180_ALT_DATA_ID;
      if (sensorInitialized && now > temperatureTime)
      {
        pressureSensor.getTemperature(temperature);
        Serial.print(temperature);
        Serial.print("     ");
        temperatureTime = now + max(pressureSensor.startTemperature(), BMP180_DATA_PERIOD);
        serial.sendData(dataId, temperature);
      }
      else
      {
        serial.sendEmpty(dataId);
        dataId = SENSOR_EMPTY_DATA_ID;
      }
      break;
    case 1:
      dataId = BMP180_VSI_DATA_ID;
      if (sensorInitialized && now > pressureTime)
      {
        pressureSensor.getPressure(pressure, temperature);
        Serial.println(pressure);
        pressureTime = now + max(pressureSensor.startPressure(3), BMP180_DATA_PERIOD);
        serial.sendData(dataId, pressure);
      }
      else
      {
        serial.sendEmpty(dataId);
        dataId = SENSOR_EMPTY_DATA_ID;
      }
      break;
    }
    sensorDataIdx++;
    if (sensorDataIdx >= BMP180_DATA_COUNT)
      sensorDataIdx = 0;
  }
  return dataId;
}

uint16_t FrSkySportSensorBMP180::decodeData(uint8_t id, uint16_t appId, uint32_t data)
{
  return SENSOR_NO_DATA_ID;
}
