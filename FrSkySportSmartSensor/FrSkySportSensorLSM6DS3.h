#ifndef _FRSKY_SPORT_SENSOR_LSM6DS3_H_
#define _FRSKY_SPORT_SENSOR_LSM6DS3_H_

#include "FrSkySportSensorOrientation.h"

class FrSkySportSensorLSM6DS3 : public FrSkySportSensorOrientation
{
  public:
    FrSkySportSensorLSM6DS3(SensorId id = ORIENTATION_DEFAULT_ID);
    void setup();
  private:
    double getRoll();
    double getPitch();
    double getGForces();
    void readSensorData();
    uint16_t getSampleRate();
};

#endif
