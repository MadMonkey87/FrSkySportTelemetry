#include "HardwareAccelerationSensor.h"

#include <MadgwickAHRS.h>

bool HardwareAccelerationSensor::IsHardwareAccelerationSensor() {
  return true;
}

double HardwareAccelerationSensor::GetGForces() {
  return sqrt(AccelerationX * AccelerationX + AccelerationY * AccelerationY + AccelerationZ * AccelerationZ) / 9.81;
}
