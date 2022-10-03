#ifndef _SBUS_Sensor_H_
#define _SBUS_Sensor_H_

#define SBUS_DATA_PERIOD 250

#include <stdint.h>
#include "HardwareSensor.h"

class SBusSensor: public HardwareSensor
{
public:
    virtual bool Setup();
    virtual void UpdateSensorData();
    virtual char* GetName();
    virtual bool IsReady();

private:
  bool Ready;
  uint32_t sbusTime;
};

#endif
