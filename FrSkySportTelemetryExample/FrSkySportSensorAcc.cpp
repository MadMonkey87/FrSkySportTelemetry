/*
  FrSky Acc-70/Acc-100 airspeed sensor clAcc for Teensy 3.x/4.0/LC, ESP8266, ATmega2560 (Mega) and ATmega328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 202000503
  Not for commercial use
*/

#include "FrSkySportSensorAcc.h"

FrSkySportSensorAcc::FrSkySportSensorAcc(SensorId id) : FrSkySportSensor(id) {}

void FrSkySportSensorAcc::setData(float x, float y, float z)
{
  xData = (uint32_t)(x * 1000.0);
  yData = (uint32_t)(y * 1000.0);
  zData = (uint32_t)(z * 1000.0);
}

void FrSkySportSensorAcc::setDataX(float x)
{
  xData = (uint32_t)(x * 1000.0);
}

void FrSkySportSensorAcc::setDataY(float y)
{
  yData = (uint32_t)(y * 1000.0);
}

void FrSkySportSensorAcc::setDataZ(float x)
{
  zData = (uint32_t)(z * 1000.0);
}

uint16_t FrSkySportSensorAcc::send(FrSkySportSingleWireSerial &serial, uint8_t id, uint32_t now)
{
  uint16_t dataId = SENSOR_NO_DATA_ID;
  if (sensorId == id)
  {
    switch (sensorDataIdx)
    {
    case 0:
      dataId = ACC_X_DATA_ID;
      if (now > accXTime)
      {
        accXTime = now + ACC_X_DATA_PERIOD;
        serial.sendData(dataId, xData);
      }
      else
      {
        serial.sendEmpty(dataId);
        dataId = SENSOR_EMPTY_DATA_ID;
      }
      break;
    case 1:
      dataId = ACC_Y_DATA_ID;
      if (now > accYTime)
      {
        accYTime = now + ACC_Y_DATA_PERIOD;
        serial.sendData(dataId, yData);
      }
      else
      {
        serial.sendEmpty(dataId);
        dataId = SENSOR_EMPTY_DATA_ID;
      }
      break;
    case 2:
      dataId = ACC_Z_DATA_ID;
      if (now > accZTime)
      {
        accZTime = now + ACC_Z_DATA_PERIOD;
        serial.sendData(dataId, zData);
      }
      else
      {
        serial.sendEmpty(dataId);
        dataId = SENSOR_EMPTY_DATA_ID;
      }
      break;
    }
    sensorDataIdx++;
    if (sensorDataIdx >= ACC_DATA_COUNT)
      sensorDataIdx = 0;
  }
  return dataId;
}

uint16_t FrSkySportSensorAcc::decodeData(uint8_t id, uint16_t appId, uint32_t data)
{
  if ((sensorId == id) || (sensorId == FrSkySportSensor::ID_IGNORE))
  {
    if (appId == ACC_X_DATA_ID)
    {
      x = data / 1000.0;
      return appId;
    }
    else if (appId == ACC_Y_DATA_ID)
    {
      y = data / 1000.0;
      return appId;
    }
    else if (appId == ACC_Z_DATA_ID)
    {
      z = data / 1000.0;
      return appId;
    }
  }
  return SENSOR_NO_DATA_ID;
}

float FrSkySportSensorAcc::getAccX() { return x; }
float FrSkySportSensorAcc::getAccY() { return y; }
float FrSkySportSensorAcc::getAccZ() { return z; }
