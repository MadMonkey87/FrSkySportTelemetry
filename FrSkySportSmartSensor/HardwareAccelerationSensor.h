#ifndef _HARDWARE_ACCELERATION_SENSOR_H_
#define _HARDWARE_ACCELERATION_SENSOR_H_

#include "HardwareSensor.h"

class HardwareAccelerationSensor: public HardwareSensor
{
  public:
    double AccelerationX, AccelerationY, AccelerationZ;
    virtual bool IsHardwareAccelerationSensor();
    double GetGForces();
};

#endif
