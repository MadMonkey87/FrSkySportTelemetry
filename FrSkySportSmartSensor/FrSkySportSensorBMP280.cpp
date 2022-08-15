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
  sensorInitialized = bmp280sensor.begin();

  if (sensorInitialized)
  {
      /* Default settings from datasheet. */
      bmp280sensor.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    /*char waitingTime = bmp280sensor.startTemperature();
    Serial.print(" - precision (1-3): ");
    Serial.println(BMP280_PRESSURE_PRECISION);
    Serial.print(" - waiting for temperature sensor (ms): ");
    Serial.println((uint16_t)waitingTime);
    delay(waitingTime);
    bmp280sensor.getTemperature(temperature);
    Serial.print(" - Temperature (C): ");
    Serial.println(temperature);

    Serial.print(" - computing baseline temperature: ");
    for (int i = 0; i < BMP280_BASELINE_SAMPLES; i++)
    {
      Serial.print(".");
      waitingTime = bmp280sensor.startTemperature();
      delay(waitingTime);
      bmp280sensor.getTemperature(temperature);
      baseLineTemperature += temperature;
    }
    baseLineTemperature = baseLineTemperature / BMP280_BASELINE_SAMPLES;
    Serial.print("\n - set as base line temperature: ");
    Serial.println(baseLineTemperature);

    waitingTime = bmp280sensor.startPressure(BMP280_PRESSURE_PRECISION);
    Serial.print(" - waiting for pressure sensor (ms): ");
    Serial.println((uint16_t)waitingTime);
    delay(waitingTime);
    bmp280sensor.getPressure(pressure, baseLineTemperature);

    Serial.print(" - Pressure (hPa): ");
    Serial.println(pressure);*/

    calibrate();

    Serial.println("done!\n");
  }
  else
  {
    Serial.println("No BMP280 sensor was found");
    Serial.print("SensorID was: 0x"); Serial.println(bmp280sensor.sensorID(),16);
    Serial.print(" - ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print(" - ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print(" - ID of 0x60 represents a BME 280.\n");
    Serial.print(" - ID of 0x61 represents a BME 680.\n");
    Serial.println("failed!\n");
  }
}

void FrSkySportSensorBMP280::calibrate()
{
  isCalibrating = true;
  /*char waitingTime;

  Serial.print(" - computing baseline pressure: ");
  for (int i = 0; i < BMP280_BASELINE_SAMPLES; i++)
  {
    Serial.print(".");
    waitingTime = bmp280sensor.startPressure(BMP280_PRESSURE_PRECISION);
    delay(waitingTime);
    bmp280sensor.getPressure(pressure, baseLineTemperature);
    baseLinePressure += pressure;
  }
  baseLinePressure = baseLinePressure / BMP280_BASELINE_SAMPLES;
  Serial.print("\n - set as base line pressure: ");
  Serial.println(baseLinePressure);

  // initialize for the loop
  waitingTime = bmp280sensor.startTemperature();
  delay(waitingTime);
  temperatureTime = millis();

  waitingTime = bmp280sensor.getPressure(pressure, baseLineTemperature);
  delay(waitingTime);
  pressureTime = millis();

  isCalibrating = false;*/
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
        /*bmp280sensor.getTemperature(temperature);
        baseLineTemperature = baseLineTemperature * 0.75 + temperature * 0.25;
        temperatureTime = now + max(bmp280sensor.startPressure(BMP280_PRESSURE_PRECISION), BMP280_DATA_PERIOD);
        serial.sendData(dataId, temperature);*/

        // Serial.print((float)baseLineTemperature, 1);
        // Serial.print("C  ");
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
        /*double oldPressureReadingTime = pressureReadingTime;

        bmp280sensor.getPressure(pressure, baseLineTemperature);
        pressureReadingTime = now;
        pressureTime = now + max(bmp280sensor.startTemperature(), BMP280_DATA_PERIOD);

        double oldRelativeAltitude = relativeAltitude;
        relativeAltitude = bmp280sensor.altitude(pressure, baseLinePressure);
        verticalSpeed = (relativeAltitude - oldRelativeAltitude) / (now - oldPressureReadingTime) * 1000;

        serial.sendData(dataId, relativeAltitude);*/

        // Serial.print((float)relativeAltitude, 1);
        // Serial.print("m  ");
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
        /*verticalSpeedTime = now + BMP280_DATA_PERIOD;
        serial.sendData(dataId, verticalSpeed);*/

        // Serial.print((float)verticalSpeed, 1);
        // Serial.println("m/s  ");
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
