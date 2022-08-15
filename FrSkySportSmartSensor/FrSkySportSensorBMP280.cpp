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

  if (bmp280sensor.begin())
  {
      /* Default settings from datasheet. */
      bmp280sensor.setSampling(
                  Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    temperature = bmp280sensor.readTemperature();
    Serial.print(" - Temperature (C): ");
    Serial.println(temperature);

    Serial.print(" - computing baseline temperature: ");
    for (int i = 0; i < BMP280_BASELINE_SAMPLES; i++)
    {
      Serial.print(".");
      baseLineTemperature += bmp280sensor.readTemperature();
    }
    baseLineTemperature = baseLineTemperature / BMP280_BASELINE_SAMPLES;
    Serial.print("\n - set as base line temperature: ");
    Serial.println(baseLineTemperature);

    pressure = bmp280sensor.readPressure();
    Serial.print(" - Pressure (hPa): ");
    Serial.println(pressure);

    calibrate();
    sensorInitialized = true;
    Serial.println("done!\n");
  }
  else
  {
    Serial.println("No BMP280 sensor was found");
    Serial.print("SensorID was: 0x"); Serial.println(bmp280sensor.sensorID(),16);
    Serial.println(" - ID of 0xFF probably means a bad address, a BMP 180 or BMP 085");
    Serial.println(" - ID of 0x56-0x58 represents a BMP 280");
    Serial.println(" - ID of 0x60 represents a BME 280");
    Serial.println(" - ID of 0x61 represents a BME 680");
    Serial.println("failed!\n");
  }
}

void FrSkySportSensorBMP280::calibrate()
{
  isCalibrating = true;
  char waitingTime;

  Serial.print(" - computing baseline pressure: ");
  for (int i = 0; i < BMP280_BASELINE_SAMPLES; i++)
  {
    Serial.print(".");
    baseLinePressure += bmp280sensor.readPressure();
  }
  baseLinePressure = baseLinePressure / BMP280_BASELINE_SAMPLES;
  Serial.print("\n - set as base line pressure: ");
  Serial.println(baseLinePressure);

  isCalibrating = false;
}

uint16_t FrSkySportSensorBMP280::send(FrSkySportSingleWireSerial &serial, uint8_t id, uint32_t now)
{
  uint16_t dataId = SENSOR_NO_DATA_ID;
  if (sensorId == id && !isCalibrating)
  {
    switch (sensorDataIdx)
    {
    case 0:
      dataId = BMP280_T_DATA_ID;
      if (sensorInitialized && now > temperatureTime)
      {
        temperature = bmp280sensor.readTemperature();
        baseLineTemperature = baseLineTemperature * 0.75 + temperature * 0.25;
        temperatureTime = now + BMP280_DATA_PERIOD;
        serial.sendData(dataId, temperature);

        Serial.print((float)baseLineTemperature, 1);
        Serial.print("C  ");
      }
      else
      {
        serial.sendEmpty(dataId);
        dataId = SENSOR_EMPTY_DATA_ID;
      }
      break;
    case 1:
      dataId = BMP280_ALT_DATA_ID;
      if (sensorInitialized && now > pressureTime)
      {
        double oldPressureReadingTime = pressureReadingTime;

        pressure = bmp280sensor.readPressure();
        pressureReadingTime = now;
        pressureTime = now + BMP280_DATA_PERIOD;

        double oldRelativeAltitude = relativeAltitude;
        relativeAltitude = -999;//bmp280sensor.altitude(pressure, baseLinePressure);
        verticalSpeed = (relativeAltitude - oldRelativeAltitude) / (now - oldPressureReadingTime) * 1000;

        serial.sendData(dataId, relativeAltitude);

        Serial.print((float)relativeAltitude, 1);
        Serial.print("m  ");
      }
      else
      {
        serial.sendEmpty(dataId);
        dataId = SENSOR_EMPTY_DATA_ID;
      }
      break;
    case 2:
      dataId = BMP280_VSI_DATA_ID;
      if (sensorInitialized && now > verticalSpeedTime)
      {
        verticalSpeedTime = now + BMP280_DATA_PERIOD;
        serial.sendData(dataId, verticalSpeed);

        Serial.print((float)verticalSpeed, 1);
        Serial.println("m/s  ");
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
