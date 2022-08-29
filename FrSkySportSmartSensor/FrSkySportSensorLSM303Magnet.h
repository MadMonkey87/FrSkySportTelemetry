#ifndef _FRSKY_SPORT_SENSOR_LSM303Magnet_H_
#define _FRSKY_SPORT_SENSOR_LSM303Magnet_H_

#include "FrSkySportSensorOrientation.h"

class FrSkySportSensorLSM303Magnet : public FrSkySportSensorOrientation
{
  public:
    FrSkySportSensorLSM303Magnet(SensorId id = ORIENTATION_DEFAULT_ID);
    void setup();

  private:
    void readSensorData();
    uint16_t getSampleRate();
};

#endif
