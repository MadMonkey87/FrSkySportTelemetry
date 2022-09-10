#include "FrSkySportSensorLSM6DS3.h"
#include <Wire.h>
#include <Arduino_LSM6DS3.h>

LSM6DS3Class sensor(Wire, 0x6B);

bool FrSkySportSensorLSM6DS3::IsReady() {
  return Ready;
}

bool FrSkySportSensorLSM6DS3::Setup()
{
  if (!sensor.begin())
  {
    return false;
  }

  Serial.print(" - Accelerometer sample rate: "); Serial.print(sensor.accelerationSampleRate()); Serial.println("Hz");
  Serial.print(" - Gyroscope sample rate: "); Serial.print(sensor.gyroscopeSampleRate()); Serial.println("Hz");

  if (!sensor.accelerationAvailable()) {
    Serial.println(" - unable to read from the acceleration sensor");
    return false;
  }

  if (!sensor.gyroscopeAvailable()) {
    Serial.println(" - unable to read from the gyro sensor");
    return false;
  }

  Ready = true;
  return true;
}

void FrSkySportSensorLSM6DS3::UpdateSensorData()
{
  if (!Ready) {
    return;
  }

  float x, y, z;

  sensor.readAcceleration(x, y, z);
  AccelerationX = x * 9.81; AccelerationY = y * 9.81; AccelerationZ = z * 9.81;

  sensor.readGyroscope(x, y, z);
  GyroX = x; GyroY = y; GyroZ = z;
}

char* FrSkySportSensorLSM6DS3::GetName() {
  return "LSM6DS3";
}
