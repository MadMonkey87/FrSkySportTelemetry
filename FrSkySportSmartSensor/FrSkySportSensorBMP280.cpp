#include "FrSkySportSensorBMP280.h"
#include <Wire.h>

bool FrSkySportSensorBMP280::Setup(){
  if (!sensor.begin(0x76)){
    return false;
  }

  sensor.setSampling(
      Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
      Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
      Adafruit_BMP280::SAMPLING_X8,     /* Pressure oversampling */
      Adafruit_BMP280::FILTER_X2,       /* Filtering. */
      Adafruit_BMP280::STANDBY_MS_250); /* Standby time. */
      
  baseAirPressure = sensor.readPressure() / 100.0;

  this->Ready = true;
  return true;
}

void FrSkySportSensorBMP280::UpdateSensorData(){
  if(!this->Ready){
    return;
  }

  AirPressure = sensor.readPressure() / 100.0;
  RelativeAltitude = sensor.readAltitude(baseAirPressure);
  Temperature = sensor.readTemperature();
}

char* FrSkySportSensorBMP280::GetName(){
  return "BMP280";
}

/*void FrSkySportSensorBMP280::setup()
{
  Serial.println("Initialize BMP280...");

  if (bmp280sensor.begin(0x76))
  {


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
}*/

/*void FrSkySportSensorBMP280::calibrate()
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
*/
