#include "FrSkySportSensorBMP180.h"

bool FrSkySportSensorBMP180::Setup(){
  Serial.println("Initialize BMP085/BMP180...");
  
  if (!sensor.begin()){
    Serial.println("failed!");
    return false;
  }
  
  Serial.print(" - Temperature: ");Serial.print(sensor.readTemperature());Serial.println(" C°");
  baseAirPressure = sensor.readPressure();
  Serial.print(" - Pressure: ");Serial.print(baseAirPressure / 100.0);Serial.println(" hPa");


  Serial.println("done!\n");
  return true;
}

void FrSkySportSensorBMP180::UpdateSensorData(){
  AirPressure = sensor.readPressure() / 100.0;
  RelativeAltitude = sensor.readAltitude(baseAirPressure);
  Temperature = sensor.readTemperature();
}

/*
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

    Serial.print(" - computing baseline temperature: ");
    for (int i = 0; i < BMP180_BASELINE_SAMPLES; i++)
    {
      Serial.print(".");
      waitingTime = bmp180sensor.startTemperature();
      delay(waitingTime);
      bmp180sensor.getTemperature(temperature);
      baseLineTemperature += temperature;
    }
    baseLineTemperature = baseLineTemperature / BMP180_BASELINE_SAMPLES;
    Serial.print("\n - set as base line temperature: ");
    Serial.println(baseLineTemperature);

    waitingTime = bmp180sensor.startPressure(BMP180_PRESSURE_PRECISION);
    Serial.print(" - waiting for pressure sensor (ms): ");
    Serial.println((uint16_t)waitingTime);
    delay(waitingTime);
    bmp180sensor.getPressure(pressure, baseLineTemperature);

    Serial.print(" - Pressure (hPa): ");
    Serial.println(pressure);

    calibrate();

    Serial.println("done!\n");
  }
  else
  {
    Serial.println("failed!\n");
  }
}

void FrSkySportSensorBMP180::calibrate()
{
  isCalibrating = true;
  char waitingTime;

  Serial.print(" - computing baseline pressure: ");
  for (int i = 0; i < BMP180_BASELINE_SAMPLES; i++)
  {
    Serial.print(".");
    waitingTime = bmp180sensor.startPressure(BMP180_PRESSURE_PRECISION);
    delay(waitingTime);
    bmp180sensor.getPressure(pressure, baseLineTemperature);
    baseLinePressure += pressure;
  }
  baseLinePressure = baseLinePressure / BMP180_BASELINE_SAMPLES;
  Serial.print("\n - set as base line pressure: ");
  Serial.println(baseLinePressure);

  // initialize for the loop
  waitingTime = bmp180sensor.startTemperature();
  delay(waitingTime);
  temperatureTime = millis();

  waitingTime = bmp180sensor.getPressure(pressure, baseLineTemperature);
  delay(waitingTime);
  pressureTime = millis();

  isCalibrating = false;
}

uint16_t FrSkySportSensorBMP180::send(FrSkySportSingleWireSerial &serial, uint8_t id, uint32_t now)
{
  uint16_t dataId = SENSOR_NO_DATA_ID;
  if (sensorId == id && !isCalibrating)
  {
    switch (sensorDataIdx)
    {
      case 0:
        dataId = BMP180_T_DATA_ID;
        if (sensorInitialized && now > temperatureTime)
        {
          bmp180sensor.getTemperature(temperature);
          baseLineTemperature = baseLineTemperature * 0.75 + temperature * 0.25;
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
        dataId = BMP180_ALT_DATA_ID;
        if (sensorInitialized && now > pressureTime)
        {
          double oldPressureReadingTime = pressureReadingTime;

          bmp180sensor.getPressure(pressure, baseLineTemperature);
          pressureReadingTime = now;
          pressureTime = now + max(bmp180sensor.startTemperature(), BMP180_DATA_PERIOD);

          double oldRelativeAltitude = relativeAltitude;
          relativeAltitude = bmp180sensor.altitude(pressure, baseLinePressure);
          verticalSpeed = (relativeAltitude - oldRelativeAltitude) / (now - oldPressureReadingTime) * 1000;
          Serial.println(relativeAltitude);
          serial.sendData(dataId, relativeAltitude);
        }
        else
        {
          serial.sendEmpty(dataId);
          dataId = SENSOR_EMPTY_DATA_ID;
        }
        break;
      case 2:
        dataId = BMP180_VSI_DATA_ID;
        if (sensorInitialized && now > verticalSpeedTime)
        {
          verticalSpeedTime = now + BMP180_DATA_PERIOD;
          serial.sendData(dataId, verticalSpeed);
        }
        else
        {
          serial.sendEmpty(dataId);
          dataId = SENSOR_EMPTY_DATA_ID;
        }
        break;
      case 3:
        dataId = BMP180_P_DATA_ID;
        if (sensorInitialized && now > rawPressureTime)
        {
          rawPressureTime = now + BMP180_DATA_PERIOD;
          serial.sendData(dataId, pressure*100);
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
*/
