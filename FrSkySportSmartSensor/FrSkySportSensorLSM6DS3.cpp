#include "FrSkySportSensorLSM6DS3.h"
#include <Arduino_LSM6DS3.h>

LSM6DS3Class lsm6ds3Sensor(Wire, 0x6B);

FrSkySportSensorLSM6DS3::FrSkySportSensorLSM6DS3(SensorId id) : FrSkySportSensorOrientation(id) {

}

void FrSkySportSensorLSM6DS3::setup()
{
  Serial.println("Initialize LSM6DS3...");
  sensorInitialized =  lsm6ds3Sensor.begin();
  if (!sensorInitialized) {
    Serial.println("failed!\n");
    return;
  }

  Serial.print(" - Accelerometer sample rate: "); Serial.print(lsm6ds3Sensor.accelerationSampleRate()); Serial.println("Hz");
  Serial.print(" - Gyroscope sample rate: "); Serial.print(lsm6ds3Sensor.gyroscopeSampleRate()); Serial.println("Hz");

  for (int i = 0; i < 10; i++) {
    if (lsm6ds3Sensor.accelerationAvailable()) {
      break;
    } else if (i >= 9) {
      sensorInitialized = false;
      Serial.println(" - unable to read from the acceleration sensor");
      Serial.println("failed!\n");
      return;
    }
    delay(100);
  }

  for (int i = 0; i < 10; i++) {
    if (lsm6ds3Sensor.gyroscopeAvailable()) {
      break;
    } else if (i >= 9) {
      sensorInitialized = false;
      Serial.println(" - unable to read from the gyro sensor");
      Serial.println("failed!\n");
      return;
    }
    delay(100);
  }

  FrSkySportSensorOrientation::setup();

  Serial.println("done!\n");
}

void FrSkySportSensorLSM6DS3::readSensorData() {
  float x, y, z;
  lsm6ds3Sensor.readAcceleration(x, y, z);
  accX = x * 9.81; accY = y * 9.81; accZ = z * 9.81;
  lsm6ds3Sensor.readGyroscope(x, y, z);
  gyroX = x; gyroY = y; gyroZ = z;
}
