#ifndef _FRSKY_SPORT_SENSOR_LSM303_H_
#define _FRSKY_SPORT_SENSOR_LSM303_H_

#include "FrSkySportSensor.h"

#define LSM303_DEFAULT_ID ID23
#define LSM303_DATA_COUNT 1
#define LSM303_X_DATA_ID 0x0700
#define LSM303_Y_DATA_ID 0x0710
#define LSM303_Z_DATA_ID 0x0720

class FrSkySportSensorLSM303 : public FrSkySportSensor
{
public:
  FrSkySportSensorLSM303(SensorId id = LSM303_DEFAULT_ID);
  void setup();
  void calibrate();
  virtual uint16_t send(FrSkySportSingleWireSerial &serial, uint8_t id, uint32_t now);
  virtual uint16_t decodeData(uint8_t id, uint16_t appId, uint32_t data);
 private:
  uint32_t accelerationTime;    // next time when the sensor is ready to provide a accelearation reading
  bool sensorInitialized;       // true if setting up the sensor was successful
};

#endif
