#include "FrSkySportSensorAltitude.h"

FrSkySportSensorAltitude::FrSkySportSensorAltitude(SensorId id) : FrSkySportSensor(id) {}

void FrSkySportSensorAltitude::Setup(HardwareAirPressureSensor* airPressureSensor) {
  this->airPressureSensor = airPressureSensor;
}

uint16_t FrSkySportSensorAltitude::send(FrSkySportSingleWireSerial& serial, uint8_t id, uint32_t now)
{
  uint16_t dataId = SENSOR_NO_DATA_ID;
  if (sensorId == id)
  {
    switch (sensorDataIdx)
    {
      case 0:
        dataId = ALTITUDE_ALT_DATA_ID;
        if (now > altitudeTime)
        {
          this->airPressureSensor->UpdateSensorData();

          verticalSpeed = airPressureSensor->RelativeAltitude - LastRelativeAltitude / (now - LastRelativeAltitude);
          LastRelativeAltitude = airPressureSensor->RelativeAltitude;
          LastReadTime = now;

          altitudeTime = now + ALTITUDE_ALT_DATA_PERIOD;
          serial.sendData(dataId, (int32_t)(this->airPressureSensor->RelativeAltitude * 100));
        }
        else
        {
          serial.sendEmpty(dataId);
          dataId = SENSOR_EMPTY_DATA_ID;
        }
        break;
      case 1:
        dataId = ALTITUDE_VSI_DATA_ID;
        if (now > vsiTime)
        {
          vsiTime = now + ALTITUDE_VSI_DATA_PERIOD;
          serial.sendData(dataId, (int32_t)(verticalSpeed * 100));
        }
        else
        {
          serial.sendEmpty(dataId);
          dataId = SENSOR_EMPTY_DATA_ID;
        }
        break;
    }
    sensorDataIdx++;
    if (sensorDataIdx >= ALTITUDE_DATA_COUNT) sensorDataIdx = 0;
  }
  return dataId;
}

uint16_t FrSkySportSensorAltitude::decodeData(uint8_t id, uint16_t appId, uint32_t data)
{
  return SENSOR_NO_DATA_ID;
}
