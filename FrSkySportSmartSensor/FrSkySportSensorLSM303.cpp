#include <Adafruit_LSM303DLH_Mag.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "FrSkySportSensorLSM303.h"

Adafruit_LSM303DLH_Mag_Unified mag = Adafruit_LSM303DLH_Mag_Unified(12345);

FrSkySportSensorLSM303::FrSkySportSensorLSM303(SensorId id) : FrSkySportSensor(id) {}

void FrSkySportSensorLSM303::setup()
{
  Serial.println("Initialize LSM303...");
  
  /* Enable auto-gain */
  mag.enableAutoRange(true);

  sensorInitialized = mag.begin();
  if(sensorInitialized)
  {
      Serial.println("done!\n");
  }
  else
  {
    Serial.println("failed!\n");
  }
}

void FrSkySportSensorLSM303::calibrate()
{
}

uint16_t FrSkySportSensorLSM303::send(FrSkySportSingleWireSerial &serial, uint8_t id, uint32_t now)
{
  uint16_t dataId = SENSOR_NO_DATA_ID;
  if (sensorId == id)
  {
    switch (sensorDataIdx)
    {
    case 0:
      dataId = LSM303_X_DATA_ID;
      if (sensorInitialized && now > accelerationTime)
      {
        sensors_event_t event;
        mag.getEvent(&event);

        dataId = SENSOR_EMPTY_DATA_ID;
        
        Serial.print("X: ");
        Serial.print(event.magnetic.x);
        Serial.print("  ");
        Serial.print("Y: ");
        Serial.print(event.magnetic.y);
        Serial.print("  ");
        Serial.print("Z: ");
        Serial.print(event.magnetic.z);
        Serial.print("  ");
        Serial.println("uT");
      }
      else
      {
        serial.sendEmpty(dataId);
        dataId = SENSOR_EMPTY_DATA_ID;
      }
      break;
    }
    sensorDataIdx++;
    if (sensorDataIdx >= LSM303_DATA_COUNT)
      sensorDataIdx = 0;
  }
  return dataId;
}

uint16_t FrSkySportSensorLSM303::decodeData(uint8_t id, uint16_t appId, uint32_t data)
{
  return SENSOR_NO_DATA_ID;
}
