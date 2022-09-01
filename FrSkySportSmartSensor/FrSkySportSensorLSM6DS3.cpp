#include "FrSkySportSensorLSM6DS3.h"

//LSM6DS3Class lsm6ds3Sensor(Wire, 0x6B);

FrSkySportSensorLSM6DS3::FrSkySportSensorLSM6DS3(){
  sensor = LSM6DS3Class(Wire, 0x6B);
}

bool FrSkySportSensorLSM6DS3::Setup()
{
  Serial.println("Initialize LSM6DS3...");

  if (!sensor.begin())
  {
    Serial.println("failed!");
    return false;
  }

  Serial.print(" - Accelerometer sample rate: "); Serial.print(lsm6ds3Sensor.accelerationSampleRate()); Serial.println("Hz");
  Serial.print(" - Gyroscope sample rate: "); Serial.print(lsm6ds3Sensor.gyroscopeSampleRate()); Serial.println("Hz");

  if(!sensor.accelerationAvailable()){
      Serial.println(" - unable to read from the acceleration sensor");
      Serial.println("failed!\n");
      return false;
  }

    if(!sensor.gyroscopeAvailable()){
      Serial.println(" - unable to read from the gyro sensor");
      Serial.println("failed!\n");
      return false;
  }

  Serial.println("done!\n");

  return true;
}

void FrSkySportSensorLSM6DS3::UpdateSensorData()
{
  float x, y, z;

  sensor.readAcceleration(x, y, z);
  AccelerationX = x * 9.81; AccelerationY = y * 9.81; AccelerationZ = z * 9.81;

  sensor.readGyroscope(x, y, z);
  GyroX = x; GyroY = y; GyroZ = z;
}