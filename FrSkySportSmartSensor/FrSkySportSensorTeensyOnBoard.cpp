#include "FrSkySportSensorTeensyOnBoard.h"
#include <Wire.h>

#include <InternalTemperature.h>

bool FrSkySportSensorTeensyOnBoard::IsReady() {
  return true;
}

bool FrSkySportSensorTeensyOnBoard::Setup() {
  return true;
}

void FrSkySportSensorTeensyOnBoard::UpdateSensorData() {
  Temperature = (double)InternalTemperature.readTemperatureC();
}

char* FrSkySportSensorTeensyOnBoard::GetName() {
  return "CPU-Temperature";
}
