#include "FrSkySportSensorBMP280.h"
#include <Wire.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp280sensor;

FrSkySportSensorBMP280::FrSkySportSensorBMP280(SensorId id) : FrSkySportSensor(id)
{
}

void FrSkySportSensorBMP280::setup()
{
  Serial.println("Initialize BMP280...");

  if (bmp280sensor.begin(0x76))
  {
    /* Default settings from datasheet. */
    bmp280sensor.setSampling(
      Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
      Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
      Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
      Adafruit_BMP280::FILTER_X16,      /* Filtering. */
      Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    temperature = bmp280sensor.readTemperature();
    Serial.print(" - temperature (C): ");
    Serial.println(temperature);

    pressure = bmp280sensor.readPressure() / 100;
    Serial.print(" - pressure (hPa): ");
    Serial.println(pressure);

    calibrate();

    sensorInitialized = true;
    Serial.println("done!\n");
  }
  else
  {
    Serial.println("No BMP280 sensor was found");
    Serial.println("failed!\n");
  }
}

void FrSkySportSensorBMP280::calibrate()
{
  baseLinePressure = bmp280sensor.readPressure() / 100;
  Serial.print(" - set as base line pressure: ");
  Serial.println(baseLinePressure);
}

uint16_t FrSkySportSensorBMP280::send(FrSkySportSingleWireSerial &serial, uint8_t id, uint32_t now)
{
  uint16_t dataId = SENSOR_NO_DATA_ID;
  if (sensorId == id)
  {
    switch (sensorDataIdx)
    {
      case 0:
        dataId = BMP280_ALT_DATA_ID;
        if (sensorInitialized && now > pressureTime)
        {
          double oldPressureReadingTime = pressureReadingTime;

          pressure = bmp280sensor.readPressure();
          pressureReadingTime = now;
          pressureTime = now + BMP280_DATA_PERIOD;

          double oldRelativeAltitude = relativeAltitude;
          relativeAltitude = bmp280sensor.readAltitude(baseLinePressure);
          verticalSpeed = (relativeAltitude - oldRelativeAltitude) / (now - oldPressureReadingTime) * 1000;

          serial.sendData(dataId, relativeAltitude);
        }
        else
        {
          serial.sendEmpty(dataId);
          dataId = SENSOR_EMPTY_DATA_ID;
        }
        break;
      case 1:
        dataId = BMP280_VSI_DATA_ID;
        if (sensorInitialized && now > verticalSpeedTime)
        {
          verticalSpeedTime = now + BMP280_DATA_PERIOD;
          serial.sendData(dataId, verticalSpeed);
        }
        else
        {
          serial.sendEmpty(dataId);
          dataId = SENSOR_EMPTY_DATA_ID;
        }
        break;
      case 2:
        dataId = BMP280_T_DATA_ID;
        if (sensorInitialized && now > temperatureTime)
        {
          temperature = bmp280sensor.readTemperature();
          temperatureTime = now + BMP280_DATA_PERIOD;
          serial.sendData(dataId, temperature);
        }
        else
        {
          serial.sendEmpty(dataId);
          dataId = SENSOR_EMPTY_DATA_ID;
        }
        break;
    }

    sensorDataIdx++;
    if (sensorDataIdx >= BMP280_DATA_COUNT)
      sensorDataIdx = 0;
  }
  return dataId;
}

uint16_t FrSkySportSensorBMP280::decodeData(uint8_t id, uint16_t appId, uint32_t data)
{
  return SENSOR_NO_DATA_ID;
}
