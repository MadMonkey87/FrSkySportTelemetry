#include "SensorTeensyOnBoard.h"
#include <Wire.h>

#include <InternalTemperature.h>

bool SensorTeensyOnBoard::IsReady() {
  return true;
}

bool SensorTeensyOnBoard::Setup() {
  return true;
}

void SensorTeensyOnBoard::UpdateSensorData() {
  Temperature = (double)InternalTemperature.readTemperatureC();
}

char* SensorTeensyOnBoard::GetName() {
  return "CPU-Temperature";
}
