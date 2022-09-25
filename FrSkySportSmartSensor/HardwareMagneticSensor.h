#ifndef _HARDWARE_MAGNETIC_SENSOR_H_
#define _HARDWARE_MAGNETIC_SENSOR_H_

#include "HardwareSensor.h"

class HardwareMagneticSensor: public HardwareSensor
{
  public:
    double MagneticX, MagneticY, MagneticZ;
    virtual bool IsHardwareMagneticSensor();
};

#endif
