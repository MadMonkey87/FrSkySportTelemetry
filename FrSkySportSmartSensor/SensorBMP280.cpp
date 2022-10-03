#include "SensorBMP280.h"
#include <Wire.h>

bool SensorBMP280::IsReady(){
  return Ready;
}

bool SensorBMP280::Setup() {
  if (!sensor.begin(0x76)) {
    return false;
  }

  sensor.setSampling(
    Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
    Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
    Adafruit_BMP280::SAMPLING_X8,     /* Pressure oversampling */
    Adafruit_BMP280::FILTER_X2,       /* Filtering. */
    Adafruit_BMP280::STANDBY_MS_250); /* Standby time. */

  baseAirPressure = sensor.readPressure() / 100.0;

  Ready = true;
  return true;
}

void SensorBMP280::UpdateSensorData() {
  if (!Ready) {
    return;
  }

  AirPressure = sensor.readPressure() / 100.0;
  RelativeAltitude = sensor.readAltitude(baseAirPressure);
  Temperature = sensor.readTemperature();
}

char* SensorBMP280::GetName() {
  return "BMP280";
}
