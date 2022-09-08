#include "FrSkySportSensorBMP180.h"

bool FrSkySportSensorBMP180::Setup() {
  if (!sensor.begin()) {
    return false;
  }

  baseAirPressure = sensor.readPressure();

  this->Ready = true;
  return true;
}

void FrSkySportSensorBMP180::UpdateSensorData() {
  if (!this->Ready) {
    return;
  }

  AirPressure = sensor.readPressure() / 100.0;
  RelativeAltitude = sensor.readAltitude(baseAirPressure);
  Temperature = sensor.readTemperature();
}

char* FrSkySportSensorBMP180::GetName() {
  return "BMP085/BMP180";
}
