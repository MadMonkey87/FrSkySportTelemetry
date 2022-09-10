#ifndef _FRSKY_SPORT_SENSOR_TEENSY_40_TEMPERATURE_H_
#define _FRSKY_SPORT_SENSOR_TEENSY_40_TEMPERATURE_H_

#include "HardwareTemperatureSensor.h"

class FrSkySportSensorTeensy40Temperature : public HardwareTemperatureSensor
{
  public:
    bool virtual Setup();
    void virtual UpdateSensorData();
    virtual char* GetName();
    virtual bool IsReady();

  private:
    bool Ready;

};

#endif
