#ifndef _FRSKY_SPORT_SENSOR_BMP180_H_
#define _FRSKY_SPORT_SENSOR_BMP180_H_

#include "FrSkySportSensor.h"

#define BMP180_DEFAULT_ID ID1
#define BMP180_DATA_COUNT 2
#define BMP180_ALT_DATA_ID 0x0100
#define BMP180_VSI_DATA_ID 0x0110

#define BMP180_DATA_PERIOD 500
#define BMP180_PRESSURE_PRECISION 3 // 1-3 where 3 is the highes precision
#define BMP180_BASELINE_SAMPLES 3

class FrSkySportSensorBMP180 : public FrSkySportSensor
{
public:
  FrSkySportSensorBMP180(SensorId id = BMP180_DEFAULT_ID);
  void setup();
  virtual uint16_t send(FrSkySportSingleWireSerial &serial, uint8_t id, uint32_t now);
  virtual uint16_t decodeData(uint8_t id, uint16_t appId, uint32_t data);

private:
  uint32_t pressureTime;
  uint32_t temperatureTime;
  bool sensorInitialized;
  double temperature;
  double pressure;
  double relativeAltitude;
  double baseLinePressure;
  double baseLineTemperature;
};

#endif
