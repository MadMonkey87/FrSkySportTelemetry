#include "FrSkySportSensorLSM6DS3.h"
#include <Kalman.h>
#include <Arduino_LSM6DS3.h>

//LSM6DS3Class lsm6ds3Sensor = LSM6DS3Class();

//LSM6DS3Class:LSM6DS3Class(Wire, 0x6B){

LSM6DS3Class lsm6ds3Sensor(Wire, 0x6B);

FrSkySportSensorLSM6DS3::FrSkySportSensorLSM6DS3(SensorId id) : FrSkySportSensor(id) {

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

  lsm6ds3Sensor.readAcceleration(accX, accY, accZ);
  lsm6ds3Sensor.readGyroscope(gyroX, gyroY, gyroZ);

  /* Set kalman and gyro starting angle */

  double roll = getRoll();
  double pitch = getPitch();

  // assumed that the initial position is the neutral position
  rollOffset = -roll;
  pitchOffset = -pitch;

  kalmanX.setAngle(roll);
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  Serial.print(" - Acceleration: "); Serial.print(accX); Serial.print("g, "); Serial.print(accY); Serial.print("g, "); Serial.print(accZ);Serial.println("g");
  Serial.print(" - Gyroscope in: "); Serial.print(gyroX); Serial.print(", "); Serial.print(gyroY); Serial.print(", "); Serial.print(gyroZ);Serial.println(" (deg/s)");
  Serial.print(" - Roll (y): "); Serial.print(roll); Serial.println("°");
  Serial.print(" - Pitch (z): "); Serial.print(pitch); Serial.println("°");

  deltaTime = micros();

  Serial.println("done!\n");
}

void FrSkySportSensorLSM6DS3::calibrate()
{
}

void FrSkySportSensorLSM6DS3::loop()
{
  readAndCalculate();
  Serial.print("LSM6S3 x: ");Serial.print(accX);Serial.print(" y: ");Serial.print(accY);Serial.print(" z:"); Serial.print(accZ);Serial.print(" rolly: ");Serial.print(kalAngleY);Serial.print(" pitchx: ");Serial.print(kalAngleX);Serial.print(" g :");Serial.println(getGForces());
}

void FrSkySportSensorLSM6DS3::readAndCalculate()
{
  if (!lsm6ds3Sensor.accelerationAvailable() || !lsm6ds3Sensor.gyroscopeAvailable()) {
    return;
  }

  lsm6ds3Sensor.readAcceleration(accX, accY, accZ);
  lsm6ds3Sensor.readGyroscope(gyroX, gyroY, gyroZ);

  double dt = (double)(micros() - deltaTime) / 1000000; // Calculate delta time
  deltaTime = micros();

  double roll = getRoll();
  double pitch = getPitch();

  gyroXrate = gyroX / 131.0; // Convert to deg/s
  gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90))
  {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  }
  else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
  {
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  }
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90))
  {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  }
  else
  {
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
  }

  if (abs(kalAngleY) > 90)
  {
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  }
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  // gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  // gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
  {
    gyroXangle = kalAngleX;
  }
  if (gyroYangle < -180 || gyroYangle > 180)
  {
    gyroYangle = kalAngleY;
  }

  /*Serial.print("AccRoll:"); Serial.print(roll); Serial.print("\t");
    Serial.print("GyroRoll:"); Serial.print(gyroXangle); Serial.print("\t");
    Serial.print("ComputedRoll:"); Serial.print(compAngleX); Serial.print("\t");
    Serial.print("KalmanRoll:"); Serial.print(kalAngleX); Serial.print("\t");
    Serial.print("final roll:"); Serial.print(kalAngleX + rollOffset); Serial.print("\t");

    Serial.println("\t");

    Serial.print("Pitch:"); Serial.print(pitch); Serial.print("\t");
    Serial.print("Gyro Y Angle:"); Serial.print(gyroYangle); Serial.print("\t");
    Serial.print("Computed Y Angle"); Serial.print(compAngleY); Serial.print("\t");
    Serial.print("Kalman Y Angle:"); Serial.print(kalAngleY); Serial.print("\t");
    Serial.print("final pitch:"); Serial.print(kalAngleY + pitchOffset); Serial.print("\t");

    Serial.println("\t");

    Serial.print(getGForces(acceleration));
    Serial.print("g");

    Serial.println("\t");
    Serial.println("\t");*/
}

uint16_t FrSkySportSensorLSM6DS3::send(FrSkySportSingleWireSerial & serial, uint8_t id, uint32_t now)
{
  uint16_t dataId = SENSOR_NO_DATA_ID;
  if (sensorId == id)
  {
    if (sensorInitialized && now > processingTime)
    {
      switch (sensorDataIdx)
      {
        case 0:
          readAndCalculate(); // do this only once for every cycle s.t all the data sent base on the same sensor readings
          dataId = LSM6DS3_GFORCE_DATA_ID;
          serial.sendData(dataId, getGForces() * 100);
          processingTime = now + LSM6DS3_PUSH_PERIOD;
          break;
        case 1:
          dataId = LSM6DS3_PITCH_DATA_ID;
          serial.sendData(dataId, (kalAngleY + pitchOffset) * 100);
          processingTime = now + LSM6DS3_PUSH_PERIOD;
          break;
        case 2:
          dataId = LSM6DS3_ROLL_DATA_ID;
          serial.sendData(dataId, (kalAngleX + rollOffset) * 100);
          processingTime = now + LSM6DS3_DATA_PERIOD;
          break;
          /*case 0:
            dataId = LSM6DS3_ACC_X_DATA_ID;
            serial.sendData(dataId, acceleration.acceleration.x * 100);
            processingTime = now + LSM6DS3_PUSH_PERIOD;
            break;
            case 1:
            dataId = LSM6DS3_ACC_Y_DATA_ID;
            serial.sendData(dataId, acceleration.acceleration.y * 100);
            processingTime = now + LSM6DS3_PUSH_PERIOD;
            break;
            case 2:
            dataId = LSM6DS3_ACC_Z_DATA_ID;
            serial.sendData(dataId, acceleration.acceleration.z * 100);
            processingTime = now + LSM6DS3_PUSH_PERIOD;
            break;

            case 6:
            dataId = LSM6DS3_PITCHSPEED_DATA_ID;
            serial.sendData(dataId, gyroXrate * 100);
            processingTime = now + LSM6DS3_PUSH_PERIOD;
            break;
            case 7:
            dataId = LSM6DS3_ROLLSPEED_DATA_ID;
            serial.sendData(dataId, gyroYrate * 100);
            processingTime = now + LSM6DS3_DATA_PERIOD;
            break;*/
      }
    }
    else
    {
      serial.sendEmpty(dataId);
      dataId = SENSOR_EMPTY_DATA_ID;
    }

    if (dataId != SENSOR_EMPTY_DATA_ID) { // some data was send, so go to the next senor value
      sensorDataIdx++;
      if (sensorDataIdx >= 3/*LSM6DS3_DATA_COUNT*/)
      {
        sensorDataIdx = 0;
      }
    }
  }
  return dataId;
}

uint16_t FrSkySportSensorLSM6DS3::decodeData(uint8_t id, uint16_t appId, uint32_t data)
{
  return SENSOR_NO_DATA_ID;
}

double FrSkySportSensorLSM6DS3::getRoll()
{
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  return atan2(accY, accZ) * RAD_TO_DEG;
#else // Eq. 28 and 29
  return atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
#endif
}

double FrSkySportSensorLSM6DS3::getPitch()
{
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  return atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  return atan2(-accX, accZ) * RAD_TO_DEG;
#endif
}

double FrSkySportSensorLSM6DS3::getGForces()
{
  return sqrt(accX * accX + accY * accY + accZ * accZ);
}
