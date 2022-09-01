#ifndef _FRSKY_SPORT_SENSOR_LSM6DS3_H_
#define _FRSKY_SPORT_SENSOR_LSM6DS3_H_

#include <Arduino_LSM6DS3.h>

#include "HardwareAccelerationSensor.h"
#include "HardwareGyroSensor.h"

class FrSkySportSensorLSM6DS3 : public HardwareAccelerationSensor, public HardwareGyroSensor
{
  public:
  FrSkySportSensorLSM6DS3();
    bool Setup();
    void UpdateSensorData();

  private:
    LSM6DS3Class sensor;

};

#endif
