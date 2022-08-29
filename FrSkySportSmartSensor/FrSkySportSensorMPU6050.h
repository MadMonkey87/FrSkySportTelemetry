#ifndef _FRSKY_SPORT_SENSOR_MPU6050_H_
#define _FRSKY_SPORT_SENSOR_MPU6050_H_

#include "FrSkySportSensorOrientation.h"

class FrSkySportSensorMPU6050 : public FrSkySportSensorOrientation
{
  public:
    FrSkySportSensorMPU6050(SensorId id = ORIENTATION_DEFAULT_ID);
    void setup();
    
  private:
    double getRoll();
    double getPitch();
    double getGForces();
    virtual void readSensorData();
};

#endif
