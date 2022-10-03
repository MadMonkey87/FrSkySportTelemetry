#ifndef _SENSOR_TEENSY_ONBOARD_H_
#define _SENSOR_TEENSY_ONBOARD_H_

#include "HardwareTemperatureSensor.h"

class SensorTeensyOnBoard : public HardwareTemperatureSensor
{
  public:
    bool virtual Setup();
    void virtual UpdateSensorData();
    virtual char* GetName();
    virtual bool IsReady();

};

#endif
