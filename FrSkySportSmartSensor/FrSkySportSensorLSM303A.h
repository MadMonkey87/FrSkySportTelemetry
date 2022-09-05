#ifndef _FRSKY_SPORT_SENSOR_LSM303A_H_
#define _FRSKY_SPORT_SENSOR_LSM303A_H_

#include <Adafruit_LSM303_U.h>

#include "HardwareAccelerationSensor.h"

class FrSkySportSensorLSM303A : public HardwareAccelerationSensor
{
  public:
    FrSkySportSensorLSM303A();
    virtual bool Setup();
    virtual void UpdateSensorData();
    virtual char* GetName();

  private:
    Adafruit_LSM303_Accel_Unified sensor;
};

#endif
