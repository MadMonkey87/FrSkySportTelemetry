#include "FrSkySportSensorOrientation.h"

FrSkySportSensorOrientation::FrSkySportSensorOrientation(SensorId id) : FrSkySportSensor(id) {}

uint16_t FrSkySportSensorOrientation::send(FrSkySportSingleWireSerial &serial, uint8_t id, uint32_t now)
{
  uint16_t dataId = SENSOR_NO_DATA_ID;
  if (sensorId == id)
  {
    if (now > processingTime)
    {
      switch (sensorDataIdx)
      {
        case 0:
          readAndCalculate(); // do this only once for every cycle s.t all the data sent base on the same sensor readings
          dataId = ORIENTATION_GFORCE_DATA_ID;
          serial.sendData(dataId, accelerationSensor->GetGForces() * 100);
          processingTime = now + ORIENTATION_PUSH_PERIOD;
          break;
        case 1:
          dataId = ORIENTATION_PITCH_DATA_ID;
          serial.sendData(dataId, (kalmanPitchAngle + pitchOffset) * 100);
          processingTime = now + ORIENTATION_PUSH_PERIOD;
          break;
        case 2:
          dataId = ORIENTATION_ROLL_DATA_ID;
          serial.sendData(dataId, (kalmanRollAngle + rollOffset) * 100);
          processingTime = now + ORIENTATION_DATA_PERIOD;
          break;
          /*case 0:
            dataId = ORIENTATION_ACC_X_DATA_ID;
            serial.sendData(dataId, acceleration.acceleration.x * 100);
            processingTime = now + ORIENTATION_PUSH_PERIOD;
            break;
            case 1:
            dataId = ORIENTATION_ACC_Y_DATA_ID;
            serial.sendData(dataId, acceleration.acceleration.y * 100);
            processingTime = now + ORIENTATION_PUSH_PERIOD;
            break;
            case 2:
            dataId = ORIENTATION_ACC_Z_DATA_ID;
            serial.sendData(dataId, acceleration.acceleration.z * 100);
            processingTime = now + ORIENTATION_PUSH_PERIOD;
            break;

            case 6:
            dataId = ORIENTATION_PITCHSPEED_DATA_ID;
            serial.sendData(dataId, gyroXrate * 100);
            processingTime = now + ORIENTATION_PUSH_PERIOD;
            break;
            case 7:
            dataId = ORIENTATION_ROLLSPEED_DATA_ID;
            serial.sendData(dataId, gyroYrate * 100);
            processingTime = now + ORIENTATION_DATA_PERIOD;
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
      if (sensorDataIdx >= 3/*ORIENTATION_DATA_COUNT*/)
      {
        sensorDataIdx = 0;
      }
    }
  }
  return dataId;
}

uint16_t FrSkySportSensorOrientation::decodeData(uint8_t id, uint16_t appId, uint32_t data)
{
  return SENSOR_NO_DATA_ID;
}

void FrSkySportSensorOrientation::Setup(HardwareAccelerationSensor* accelerationSensor, HardwareGyroSensor* gyroSensor, HardwareMagneticSensor* magneticSensor =  NULL) {
  this->accelerationSensor = accelerationSensor;
  this->gyroSensor = gyroSensor;
  this->magneticSensor = magneticSensor;

  /*readSensorData();

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

    filter.begin(getSampleRate());
    Serial.print(" - Sensor sample rate: "); Serial.print(getSampleRate()); Serial.println(" Hz");
    Serial.print(" - Acceleration: "); Serial.print(accX); Serial.print(", "); Serial.print(accY); Serial.print(", "); Serial.print(accZ); Serial.println(" (m/s2)");
    Serial.print(" - Gyroscope: "); Serial.print(gyroX); Serial.print(", "); Serial.print(gyroY); Serial.print(", "); Serial.print(gyroZ); Serial.println(" (rad/s)");
    Serial.print(" - Magnetometer: "); Serial.print(magnetometerX); Serial.print(", "); Serial.print(magnetometerY); Serial.print(", "); Serial.print(magnetometerZ); Serial.println(" (uT)");
    Serial.print(" - Roll (y): "); Serial.print(roll); Serial.println("°");
    Serial.print(" - Pitch (z): "); Serial.print(pitch); Serial.println("°");

    deltaTime = micros();*/
}

void FrSkySportSensorOrientation::loop()
{
  /*readAndCalculate();
    Serial.print("x: "); Serial.print(accX); Serial.print(" y: "); Serial.print(accY); Serial.print(" z:"); Serial.print(accZ); Serial.print(" rolly: "); Serial.print(kalAngleY); Serial.print(" pitchx: "); Serial.print(kalAngleX); Serial.print(" g :"); Serial.println(getGForces());
    Serial.print("roll: "); Serial.print(filter.getRoll());
    Serial.print("pitch: "); Serial.print(filter.getPitch());
    Serial.print("yaw: "); Serial.println(filter.getYaw());
    Serial.println("");*/

  //readSensorData();

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = 0;
  if (magneticSensor && magneticSensor->IsReady()) {
    magneticSensor->UpdateSensorData();
    heading = atan2(magneticSensor->MagneticY, magneticSensor->MagneticX);
  }

  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.22;
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if (heading < 0)
    heading += 2 * PI;

  // Check for wrap due to addition of declination.
  if (heading > 2 * PI)
    heading -= 2 * PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180 / M_PI;
  Serial.print("heading: ");
  Serial.print(headingDegrees);
  //Serial.print("° x:"); Serial.print(magnetometerX); Serial.print(" y:"); Serial.print(magnetometerY); Serial.print(" z:"); Serial.println(magnetometerY);
}

void FrSkySportSensorOrientation::readAndCalculate()
{
  if (accelerationSensor && accelerationSensor->IsReady()) {
    accelerationSensor->UpdateSensorData();
  } else {
    Serial.println("No acceleration sensor or sensor not ready");
    return;
  }

  if (gyroSensor && gyroSensor->IsReady()) {
    gyroSensor->UpdateSensorData();
  } else {
    Serial.println("No gyro sensor or sensor not ready");
    return;
  }

  if (magneticSensor && magneticSensor->IsReady()) {
    magneticSensor->UpdateSensorData();
    filter.update(gyroSensor->GyroX, gyroSensor->GyroY, gyroSensor->GyroZ, accelerationSensor->AccelerationX, accelerationSensor->AccelerationY, accelerationSensor->AccelerationZ, magneticSensor->MagneticX, magneticSensor->MagneticY, magneticSensor->MagneticZ);
  } else {
    filter.updateIMU(gyroSensor->GyroX, gyroSensor->GyroY, gyroSensor->GyroZ, accelerationSensor->AccelerationX, accelerationSensor->AccelerationY, accelerationSensor->AccelerationZ);
  }

  double dt = (double)(micros() - deltaTime) / 1000000; // Calculate delta time
  deltaTime = micros();

  accelerationRollAngle = getRollFromAcceleration();
  accelerationPitchAngle = getPitchFromAcceleration();

  gyroRollRate = gyroSensor->GyroX / 131.0; // Convert to deg/s
  gyroPitchRate = gyroSensor->GyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((accelerationRollAngle < -90 && kalAngleX > 90) || (accelerationRollAngle > 90 && kalAngleX < -90))
  {
    kalmanX.setAngle(accelerationRollAngle);
    compAngleX = accelerationRollAngle;
    kalAngleX = accelerationRollAngle;
    gyroXangle = accelerationRollAngle;
  }
  else
    kalAngleX = kalmanX.getAngle(accelerationRollAngle, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
  {
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  }
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((accelerationPitchAngle < -90 && kalmanPitchAngle > 90) || (accelerationPitchAngle > 90 && kalmanPitchAngle < -90))
  {
    kalmanPitch.setAngle(accelerationPitchAngle);
    complementaryPitchAngle = accelerationPitchAngle;
    kalmanPitchAngle = accelerationPitchAngle;
    gyroRollAngle = accelerationPitchAngle;
  }
  else
  {
    kalmanPitchAngle = kalmanPitch.getAngle(accelerationPitchAngle, gyroPitchRate, dt); // Calculate the angle using a Kalman filter
  }

  if (abs(kalmanPitchAngle) > 90)
  {
    gyroRollRate = -gyroRollRate; // Invert rate, so it fits the restriced accelerometer reading
  }
  kalmanRollAngle = kalmanRoll.getAngle(accelerationRollAngle, gyroRollRate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroRollRate += gyroRollRate * dt; // Calculate gyro angle without any filter
  gyroPitchRate += gyroPitchRate * dt;
  // gyroRollRate += kalmanRoll.getRate() * dt; // Calculate gyro angle using the unbiased rate
  // gyroPitchRate += kalmanPitch.getRate() * dt;

  complementaryRollAngle = 0.93 * (complementaryRollAngle + gyroRollRate * dt) + 0.07 * accelerationRollAngle; // Calculate the angle using a Complimentary filter
  complementaryPitchAngle = 0.93 * (complementaryPitchAngle + gyroPitchRate * dt) + 0.07 * accelerationPitchAngle;

  // Reset the gyro angle when it has drifted too much
  if (gyroPitchAngle < -180 || gyroPitchAngle > 180)
  {
    gyroPitchAngle = kalmanRollAngle;
  }
  if (gyroRollAngle < -180 || gyroRollAngle > 180)
  {
    gyroRollAngle = kalmanPitchAngle;
  }

  Serial.print("pitch:");
  Serial.print(accelerationPitchAngle);
  Serial.print(",roll:");
  Serial.print(accelerationRollAngle);

  Serial.print("kalman_pitch:");
  Serial.print(kalmanPitchAngle);
  Serial.print(",kalman_roll:");
  Serial.print(kalmanRollAngle);

  Serial.print("AHRS_pitch:");
  Serial.print(filter.getPitch());
  Serial.print(",AHRS_roll:");
  Serial.print(filter.getRoll());

  Serial.print("complementary_pitch:");
  Serial.print(complementaryPitchAngle);
  Serial.print(",complementary_roll:");
  Serial.println(complementaryRollAngle);
}

double FrSkySportSensorOrientation::getRollFromAcceleration()
{
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  return atan2(accelerationSensor->AccelerationY, accelerationSensor->AccelerationZ) * RAD_TO_DEG;
#else // Eq. 28 and 29
  return atan(accelerationSensor->AccelerationY / sqrt(accelerationSensor->AccelerationX * accelerationSensor->AccelerationX + accelerationSensor->AccelerationZ * accelerationSensor->AccelerationZ)) * RAD_TO_DEG;
#endif
}

double FrSkySportSensorOrientation::getPitchFromAcceleration()
{
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  return atan(-accelerationSensor->AccelerationX / sqrt(accelerationSensor->AccelerationY * accelerationSensor->AccelerationY + accelerationSensor->AccelerationZ * accelerationSensor->AccelerationZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  return atan2(-accelerationSensor->AccelerationX, accelerationSensor->AccelerationZ) * RAD_TO_DEG;
#endif
}
