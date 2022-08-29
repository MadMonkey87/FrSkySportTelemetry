#include "FrSkySportSensorMPU6050.h"
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;

FrSkySportSensorMPU6050::FrSkySportSensorMPU6050(SensorId id) : FrSkySportSensorOrientation(id) {}

void FrSkySportSensorMPU6050::setup()
{
  Serial.println("Initialize MPU6050...");

  sensorInitialized = mpu.begin();
  if (sensorInitialized)
  {
    mpu_temp = mpu.getTemperatureSensor();
    mpu_temp->printSensorDetails();

    mpu_accel = mpu.getAccelerometerSensor();
    mpu_accel->printSensorDetails();

    mpu_gyro = mpu.getGyroSensor();
    mpu_gyro->printSensorDetails();

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    Serial.print(" - Accelerometer range set to: ");
    switch (mpu.getAccelerometerRange())
    {
      case MPU6050_RANGE_2_G:
        Serial.println("+-2G");
        break;
      case MPU6050_RANGE_4_G:
        Serial.println("+-4G");
        break;
      case MPU6050_RANGE_8_G:
        Serial.println("+-8G");
        break;
      case MPU6050_RANGE_16_G:
        Serial.println("+-16G");
        break;
    }
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    Serial.print(" - Gyro range set to: ");
    switch (mpu.getGyroRange())
    {
      case MPU6050_RANGE_250_DEG:
        Serial.println("+- 250 deg/s");
        break;
      case MPU6050_RANGE_500_DEG:
        Serial.println("+- 500 deg/s");
        break;
      case MPU6050_RANGE_1000_DEG:
        Serial.println("+- 1000 deg/s");
        break;
      case MPU6050_RANGE_2000_DEG:
        Serial.println("+- 2000 deg/s");
        break;
    }

    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.print(" - Filter bandwidth set to: ");
    switch (mpu.getFilterBandwidth())
    {
      case MPU6050_BAND_260_HZ:
        Serial.println("260 Hz");
        break;
      case MPU6050_BAND_184_HZ:
        Serial.println("184 Hz");
        break;
      case MPU6050_BAND_94_HZ:
        Serial.println("94 Hz");
        break;
      case MPU6050_BAND_44_HZ:
        Serial.println("44 Hz");
        break;
      case MPU6050_BAND_21_HZ:
        Serial.println("21 Hz");
        break;
      case MPU6050_BAND_10_HZ:
        Serial.println("10 Hz");
        break;
      case MPU6050_BAND_5_HZ:
        Serial.println("5 Hz");
        break;
    }

    FrSkySportSensorOrientation::setup();

    Serial.println("done!\n");
  }
  else
  {
    Serial.println("failed!\n");
  }
}

void FrSkySportSensorMPU6050::readSensorData() {
  sensors_event_t acceleration;
  mpu_accel->getEvent(&acceleration);
  accX = acceleration.acceleration.x;
  accY = acceleration.acceleration.y;
  accZ = acceleration.acceleration.z;

  sensors_event_t gyro;
  mpu_gyro->getEvent(&gyro);
  gyroX = gyro.acceleration.x;
  gyroY = gyro.acceleration.y;
  gyroZ = gyro.acceleration.z;
}

uint16_t FrSkySportSensorMPU6050::getSampleRate() {
  switch (mpu.getFilterBandwidth())
  {
    case MPU6050_BAND_260_HZ:
      return 260;
    case MPU6050_BAND_184_HZ:
      return 184;
    case MPU6050_BAND_94_HZ:
      return 94;
    case MPU6050_BAND_44_HZ:
      return 44;
    case MPU6050_BAND_21_HZ:
      return 21;
    case MPU6050_BAND_10_HZ:
      return 10;
    case MPU6050_BAND_5_HZ:
      return 5;
  }
  return 0;
}
