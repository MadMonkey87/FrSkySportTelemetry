#ifndef _SENSOR_LSM303M_H_
#define _SENSOR_LSM303M_H_

#include <Adafruit_LSM303_U.h>

#include "HardwareMagneticSensor.h"

class SensorLSM303M : public HardwareMagneticSensor
{
  public:
    SensorLSM303M();
    virtual bool Setup();
    virtual void UpdateSensorData();
    virtual char* GetName();
    virtual bool IsReady();

  private:
    Adafruit_LSM303_Mag_Unified sensor;
    bool Ready;
};

#endif
