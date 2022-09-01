#ifndef _FRSKY_SPORT_SENSOR_HMC5883L_H_
#define _FRSKY_SPORT_SENSOR_HMC5883L_H_

#include "FrSkySportSensorOrientation.h"

class FrSkySportSensorHMC5883L : public FrSkySportSensorOrientation
{
  public:
    FrSkySportSensorHMC5883L(SensorId id = ORIENTATION_DEFAULT_ID);
    void setup();

  private:
    void readSensorData();
    uint16_t getSampleRate();
};

#endif
