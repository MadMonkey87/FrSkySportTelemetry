#ifndef _FRSKY_SPORT_SENSOR_LSM303M_H_
#define _FRSKY_SPORT_SENSOR_LSM303M_H_

#include <Adafruit_LSM303_U.h>

#include "HardwareMagneticSensor.h"

class FrSkySportSensorLSM303M : public HardwareMagneticSensor
{
  public:
    FrSkySportSensorLSM303M();
    virtual bool Setup();
    virtual void UpdateSensorData();

  private:
    Adafruit_LSM303_Mag_Unified sensor;
};

#endif
