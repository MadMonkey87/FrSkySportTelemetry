#include "FrSkySportSensorMPU6050.h"
#include <Adafruit_MPU6050.h>
#include <Kalman.h>

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;

FrSkySportSensorMPU6050::FrSkySportSensorMPU6050(SensorId id) : FrSkySportSensor(id) {}

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
    Serial.print("Accelerometer range set to: ");
    switch (mpu.getAccelerometerRange()) {
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
    Serial.print("Gyro range set to: ");
    switch (mpu.getGyroRange()) {
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
    Serial.print("Filter bandwidth set to: ");
    switch (mpu.getFilterBandwidth()) {
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

      delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
   sensors_event_t accel;
mpu_accel->getEvent(&accel);
  accX = accel.acceleration.x;
  accY = accel.acceleration.y;
  accZ = accel.acceleration.z;

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();

    Serial.println("\ndone!\n");
  }
  else
  {
    Serial.println("failed!\n");
  }
}

void FrSkySportSensorMPU6050::calibrate()
{
}

void FrSkySportSensorMPU6050::loop()
{
    sensors_event_t accel;
    sensors_event_t gyro;
    mpu_accel->getEvent(&accel);
    mpu_gyro->getEvent(&gyro);

      double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

    accX = accel.acceleration.x;
    accY = accel.acceleration.y;
    accZ = accel.acceleration.z;

    gyroX = gyro.gyro.x;
    gyroY = gyro.gyro.y;
    gyroZ = gyro.gyro.z;

    #ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

  #ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif


  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;









#if 1 // Set to 1 to activate
  Serial.print("Acc X:"); Serial.print(accX); Serial.print("\t");
  Serial.print("Acc Y:");Serial.print(accY); Serial.print("\t");
  Serial.print("Acc Z:");Serial.print(accZ); Serial.print("\t");

  Serial.print("Gy X:");Serial.print(gyroX); Serial.print("\t");
  Serial.print("Gy X:");Serial.print(gyroY); Serial.print("\t");
  Serial.print("Gy X:");Serial.print(gyroZ); Serial.print("\t");

  Serial.println("\t");
#endif

  Serial.print("Roll:");Serial.print(roll); Serial.print("\t");
  Serial.print("Gyro X Angle:");Serial.print(gyroXangle); Serial.print("\t");
  Serial.print("Computed X Angle:"); Serial.print(compAngleX); Serial.print("\t");
  Serial.print("Kalman X Angle:"); Serial.print(kalAngleX); Serial.print("\t");

  Serial.println("\t");

  Serial.print("Pitch:");Serial.print(pitch); Serial.print("\t");
  Serial.print("Gyro Y Angle:");Serial.print(gyroYangle); Serial.print("\t");
  Serial.print("Computed Y Angle");Serial.print(compAngleY); Serial.print("\t");
  Serial.print("Kalman Y Anggle:");Serial.print(kalAngleY); Serial.print("\t");
 Serial.println("\t");
  Serial.println("\t");




  


    delay(100);
}

uint16_t FrSkySportSensorMPU6050::send(FrSkySportSingleWireSerial &serial, uint8_t id, uint32_t now)
{
  uint16_t dataId = SENSOR_NO_DATA_ID;
  if (sensorId == id)
  {
    switch (sensorDataIdx)
    {
      case 0:
        dataId = MPU6050_X_DATA_ID;
        if (sensorInitialized && now > accelerationTime)
        {

        }
        else
        {
          serial.sendEmpty(dataId);
          dataId = SENSOR_EMPTY_DATA_ID;
        }
        break;
    }
    sensorDataIdx++;
    if (sensorDataIdx >= MPU6050_DATA_COUNT)
      sensorDataIdx = 0;
  }
  return dataId;
}

uint16_t FrSkySportSensorMPU6050::decodeData(uint8_t id, uint16_t appId, uint32_t data)
{
  return SENSOR_NO_DATA_ID;
}
