#include "HardwareSensor.h"

bool HardwareSensor::Setup() {}

void HardwareSensor::UpdateSensorData() {}

char* HardwareSensor::GetName() {}

bool HardwareSensor::IsHardwareTemperatureSensor() {
  return false;
}

bool HardwareSensor::IsHardwareAccelerationSensor() {
  return false;
}

bool HardwareSensor::IsHardwareGyroSensor() {
  return false;
}

bool HardwareSensor::IsHardwareMagneticSensor() {
  return false;
}

bool HardwareSensor::IsHardwareAirPressureSensor() {
  return false;
}
