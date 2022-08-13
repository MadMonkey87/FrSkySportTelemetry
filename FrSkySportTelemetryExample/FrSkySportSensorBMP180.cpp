#include "FrSkySportSensorBMP180.h"
#include <SFE_BMP180.h>
#include <Wire.h>
SFE_BMP180 bmp180sensor;
#define ALTITUDE 548.0 // base altitude

FrSkySportSensorBMP180::FrSkySportSensorBMP180(SensorId id) : FrSkySportSensor(id)
{
}

void FrSkySportSensorBMP180::setup()
{
  Serial.println("Initialize BMP180...");
  sensorInitialized = bmp180sensor.begin();

  if (sensorInitialized)
  {
    char waitingTime = bmp180sensor.startTemperature();
    Serial.print(" - precision (1-3): ");
    Serial.println(BMP180_PRESSURE_PRECISION);
    Serial.print(" - waiting for temperature sensor (ms): ");
    Serial.println((uint16_t)waitingTime);
    delay(waitingTime);
    bmp180sensor.getTemperature(temperature);
    Serial.print(" - Temperature (C): ");
    Serial.println(temperature);
    
    waitingTime = bmp180sensor.startPressure(BMP180_PRESSURE_PRECISION);
    Serial.print(" - waiting for pressure sensor (ms): ");
    Serial.println((uint16_t)waitingTime);
    delay(waitingTime);
    bmp180sensor.getPressure(pressure, temperature);
    
    Serial.print(" - Pressure (hPa): ");
    Serial.println(pressure);
    
    baseLinePressure = pressure;
    Serial.println(" - set as base line pressure");
    

    //initialize for the loop
    waitingTime = bmp180sensor.startTemperature();
    temperatureTime = millis() + waitingTime;
    
    Serial.println("done!\n");
  }
  else
  {
    Serial.println("failed!\n");
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
        bmp180sensor.getTemperature(temperature);
        //Serial.print(temperature);
        //Serial.print("     ");
        temperatureTime = now + max(bmp180sensor.startPressure(BMP180_PRESSURE_PRECISION), BMP180_DATA_PERIOD);
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
        bmp180sensor.getPressure(pressure, temperature);
        //Serial.println(pressure);
        pressureTime = now + max(bmp180sensor.startTemperature(), BMP180_DATA_PERIOD);
        Serial.println(bmp180sensor.altitude(pressure, baseLinePressure));
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
