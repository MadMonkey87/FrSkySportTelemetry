#ifndef _SENSOR_LSM6DS3_H_
#define _SENSOR_LSM6DS3_H_

#include "HardwareAccelerationSensor.h"
#include "HardwareGyroSensor.h"

class SensorLSM6DS3 : public HardwareAccelerationSensor, public HardwareGyroSensor
{
  public:
    virtual bool Setup();
    virtual void UpdateSensorData();
    virtual char* GetName();
    virtual bool IsReady();

  private:
    bool Ready;

};

#endif
