#ifndef _FRSKY_SPORT_SENSOR_TEENSY_40_TEMPERATURE_H_
#define _FRSKY_SPORT_SENSOR_TEENSY_40_TEMPERATURE_H_

#include "HardwareTemperatureSensor.h"

class FrSkySportSensorTeensy40Temperature : public HardwareTemperatureSensor
{
  public:
    bool Setup();
    void UpdateSensorData();
    bool Ready;
};

#endif
