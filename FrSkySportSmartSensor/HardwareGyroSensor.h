#ifndef _HARDWARE_GYRO_SENSOR_H_
#define _HARDWARE_GYRO_SENSOR_H_

#include "HardwareSensor.h"

class HardwareGyroSensor: public HardwareSensor
{
  public:
    double GyroX, GyroY, GyroZ; 

    virtual bool IsHardwareGyroSensor();

};

#endif
