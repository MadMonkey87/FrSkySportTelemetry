#include "SensorMPU6050.h"

bool SensorMPU6050::IsReady() {
  return Ready;
}

bool SensorMPU6050::Setup()
{
  if (!mpu.begin())
  {
    return false;
  }

  temperatureSensor = mpu.getTemperatureSensor();
  temperatureSensor->printSensorDetails();

  accelerationSensor = mpu.getAccelerometerSensor();
  accelerationSensor->printSensorDetails();

  gyroSensor = mpu.getGyroSensor();
  gyroSensor->printSensorDetails();

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

  Ready = true;
  return true;
}

void SensorMPU6050::UpdateSensorData()
{
  if (!Ready) {
    return;
  }

  sensors_event_t event;

  accelerationSensor->getEvent(&event);
  AccelerationX = event.acceleration.x;
  AccelerationY = event.acceleration.y;
  AccelerationZ = event.acceleration.z;

  gyroSensor->getEvent(&event);
  GyroX = event.gyro.x;
  GyroY = event.gyro.y;
  GyroZ = event.gyro.z;

  temperatureSensor->getEvent(&event);
  Temperature = event.temperature;
}

char* SensorMPU6050::GetName() {
  return "MPU6050";
}
