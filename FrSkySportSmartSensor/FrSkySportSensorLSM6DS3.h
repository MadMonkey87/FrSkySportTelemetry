#ifndef _FRSKY_SPORT_SENSOR_LSM6DS3_H_
#define _FRSKY_SPORT_SENSOR_LSM6DS3_H_

#include "HardwareAccelerationSensor.h"
#include "HardwareGyroSensor.h"

class FrSkySportSensorLSM6DS3 : public HardwareAccelerationSensor, public HardwareGyroSensor
{
  public:
    virtual bool Setup();
    virtual void UpdateSensorData();
    bool Ready;
};

#endif
