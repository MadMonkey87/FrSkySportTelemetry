#include "FrSkySportSensorOrientation.h"

FrSkySportSensorOrientation::FrSkySportSensorOrientation(SensorId id) : FrSkySportSensor(id) {}

uint16_t FrSkySportSensorOrientation::send(FrSkySportSingleWireSerial &serial, uint8_t id, uint32_t now)
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
          dataId = ORIENTATION_GFORCE_DATA_ID;
          serial.sendData(dataId, getGForces() * 100);
          processingTime = now + ORIENTATION_PUSH_PERIOD;
          break;
        case 1:
          dataId = ORIENTATION_PITCH_DATA_ID;
          serial.sendData(dataId, (kalAngleY + pitchOffset) * 100);
          processingTime = now + ORIENTATION_PUSH_PERIOD;
          break;
        case 2:
          dataId = ORIENTATION_ROLL_DATA_ID;
          serial.sendData(dataId, (kalAngleX + rollOffset) * 100);
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
  if(!sensorInitialized){
    return;
  }
  /*readAndCalculate();
  Serial.print("x: "); Serial.print(accX); Serial.print(" y: "); Serial.print(accY); Serial.print(" z:"); Serial.print(accZ); Serial.print(" rolly: "); Serial.print(kalAngleY); Serial.print(" pitchx: "); Serial.print(kalAngleX); Serial.print(" g :"); Serial.println(getGForces());
  Serial.print("roll: "); Serial.print(filter.getRoll());
  Serial.print("pitch: "); Serial.print(filter.getPitch());
  Serial.print("yaw: "); Serial.println(filter.getYaw());
  Serial.println("");*/

  //readSensorData();

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(magnetometerY, magnetometerX);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.22;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  Serial.print("heading: ");
  Serial.print(headingDegrees);
  Serial.print("° x:");Serial.print(magnetometerX);Serial.print(" y:");Serial.print(magnetometerY);Serial.print(" z:");Serial.println(magnetometerY);
}

void FrSkySportSensorOrientation::readAndCalculate()
{
  //readSensorData();

  double dt = (double)(micros() - deltaTime) / 1000000; // Calculate delta time
  deltaTime = micros();

  filter.updateIMU(gyroX, gyroY, gyroZ, accX, accY, accZ);

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
}

double FrSkySportSensorOrientation::getRoll()
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

double FrSkySportSensorOrientation::getPitch()
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

double FrSkySportSensorOrientation::getGForces()
{
  return sqrt(accX * accX + accY * accY + accZ * accZ) / 9.81;
}
