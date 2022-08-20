#ifndef _FRSKY_SPORT_SENSOR_MPU6050_H_
#define _FRSKY_SPORT_SENSOR_MPU6050_H_

#include "FrSkySportSensor.h"

#define MPU6050_DEFAULT_ID ID23
#define MPU6050_DATA_COUNT 1
#define MPU6050_X_DATA_ID 0x0700
#define MPU6050_Y_DATA_ID 0x0710
#define MPU6050_Z_DATA_ID 0x0720

class FrSkySportSensorMPU6050 : public FrSkySportSensor
{
public:
  FrSkySportSensorMPU6050(SensorId id = MPU6050_DEFAULT_ID);
  void setup();
  void calibrate();
  virtual uint16_t send(FrSkySportSingleWireSerial &serial, uint8_t id, uint32_t now);
  virtual uint16_t decodeData(uint8_t id, uint16_t appId, uint32_t data);
 private:
  uint32_t accelerationTime;    // next time when the sensor is ready to provide a accelearation reading
  bool sensorInitialized;       // true if setting up the sensor was successful
};

#endif
