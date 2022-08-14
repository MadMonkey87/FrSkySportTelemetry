#ifndef _FRSKY_SPORT_SENSOR_BMP180_H_
#define _FRSKY_SPORT_SENSOR_BMP180_H_

#include "FrSkySportSensor.h"

#define BMP180_DEFAULT_ID ID1
#define BMP180_DATA_COUNT 3       // temperature, relative altitude, vertical speed
#define BMP180_ALT_DATA_ID 0x0100 // relative altitude to the start altitude in m
#define BMP180_VSI_DATA_ID 0x0110 // vertical speed in m/s
#define BMP180_T_DATA_ID 0x0400   // temperature in C

#define BMP180_DATA_PERIOD 500
#define BMP180_PRESSURE_PRECISION 3 // 1-3 where 3 is the highes precision
#define BMP180_BASELINE_SAMPLES 3

class FrSkySportSensorBMP180 : public FrSkySportSensor
{
public:
  FrSkySportSensorBMP180(SensorId id = BMP180_DEFAULT_ID);
  void setup();
  void calibrate();
  virtual uint16_t send(FrSkySportSingleWireSerial &serial, uint8_t id, uint32_t now);
  virtual uint16_t decodeData(uint8_t id, uint16_t appId, uint32_t data);

private:
  uint32_t pressureTime;    // next time when the sensor is ready to provide a pressure reading
  uint32_t temperatureTime; // next time when the sensor is ready to provide a temperature reading
  uint32_t verticalSpeedTime;
  uint32_t pressureReadingTime; // time when the last preassure measurement was registered
  bool sensorInitialized;       // true if setting up the sensor was successful
  double temperature;           // reading from the sensor
  double pressure;              // reading from the sensor
  double verticalSpeed;         // calculated
  double relativeAltitude;      // calculated from the baseLinePressure and the current preassure reading
  double baseLinePressure;      // zero level pressure at bootup
  double baseLineTemperature;
  bool isCalibrating;
};

#endif
