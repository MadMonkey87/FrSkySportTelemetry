#ifndef _HARDWARE_AIR_PRESSURE_SENSOR_H_
#define _HARDWARE_AIR_PRESSURE_SENSOR_H_

#include "HardwareSensor.h"

class HardwareAirPressureSensor: public HardwareSensor
{
  public:
    double AirPressure; // in hPa
    double RelativeAltitude; //in m
};

#endif
