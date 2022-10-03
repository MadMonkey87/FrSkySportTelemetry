#ifndef _SENSOR_HMC5883L_H_
#define _SENSOR_HMC5883L_H_

#include <Adafruit_HMC5883_U.h>

#include "HardwareMagneticSensor.h"

class SensorHMC5883L : public HardwareMagneticSensor
{
  public:
    SensorHMC5883L();
    virtual bool Setup();
    virtual void UpdateSensorData();
    virtual char* GetName();
    virtual bool IsReady();

  private:
    Adafruit_HMC5883_Unified sensor;
    bool Ready;
};

#endif
