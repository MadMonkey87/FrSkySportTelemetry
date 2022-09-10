#ifndef _FRSKY_SPORT_SENSOR_TEENSY_ONBOARD_H_
#define _FRSKY_SPORT_SENSOR_TEENSY_ONBOARD_H_

#include "HardwareTemperatureSensor.h"

class FrSkySportSensorTeensyOnBoard : public HardwareTemperatureSensor
{
  public:
    bool virtual Setup();
    void virtual UpdateSensorData();
    virtual char* GetName();
    virtual bool IsReady();

};

#endif
