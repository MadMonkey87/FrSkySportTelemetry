#ifndef _FRSKY_SPORT_SENSOR_HMC5883L_H_
#define _FRSKY_SPORT_SENSOR_HMC5883L_H_

#include <Adafruit_HMC5883_U.h>

#include "HardwareMagneticSensor.h"

class FrSkySportSensorHMC5883L : public HardwareMagneticSensor
{
  public:
    FrSkySportSensorHMC5883L();
    bool Setup();
    void UpdateSensorData();

  private:
    Adafruit_HMC5883_Unified sensor;
};

#endif
