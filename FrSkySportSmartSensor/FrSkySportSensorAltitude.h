#ifndef _FRSKY_SPORT_SENSOR_ALTITUDE_H_
#define _FRSKY_SPORT_SENSOR_ALTITUDE_H_

#include "FrSkySportSensor.h"
#include "HardwareAirPressureSensor.h"

#define ALTITUDE_DEFAULT_ID ID1
#define ALTITUDE_DATA_COUNT 2
#define ALTITUDE_ALT_DATA_ID 0x0100
#define ALTITUDE_VSI_DATA_ID 0x0110

#define ALTITUDE_ALT_DATA_PERIOD 200
#define ALTITUDE_VSI_DATA_PERIOD 100

class FrSkySportSensorAltitude : public FrSkySportSensor
{
  public:
    FrSkySportSensorAltitude(SensorId id = ALTITUDE_DEFAULT_ID);
    void Setup(HardwareAirPressureSensor* airPressureSensor);
    virtual uint16_t send(FrSkySportSingleWireSerial& serial, uint8_t id, uint32_t now);
    virtual uint16_t decodeData(uint8_t id, uint16_t appId, uint32_t data);

  private:
    uint32_t altitudeTime;
    uint32_t vsiTime;
    HardwareAirPressureSensor* airPressureSensor;
    double verticalSpeed;

    // used to calculate the vertical speed
    double LastRelativeAltitude;
    uint32_t LastReadTime;
};

#endif
