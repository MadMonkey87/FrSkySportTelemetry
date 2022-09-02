#ifndef _HARDWARE_TEMPERATURE_SENSOR_H_
#define _HARDWARE_TEMPERATURE_SENSOR_H_

#include "HardwareSensor.h"

class HardwareTemperatureSensor: public HardwareSensor
{
  public:
    double Temperature; // in C
};

#endif
