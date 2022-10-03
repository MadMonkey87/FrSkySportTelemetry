#include "SensorBMP180.h"

bool SensorBMP180::IsReady(){
  return Ready;
}

bool SensorBMP180::Setup() {
  if (!sensor.begin()) {
    return false;
  }

  baseAirPressure = sensor.readPressure();

  Ready = true;
  return true;
}

void SensorBMP180::UpdateSensorData() {
  if (Ready) {
    return;
  }

  AirPressure = sensor.readPressure() / 100.0;
  RelativeAltitude = sensor.readAltitude(baseAirPressure);
  Temperature = sensor.readTemperature();
}

char* SensorBMP180::GetName() {
  return "BMP085/BMP180";
}
