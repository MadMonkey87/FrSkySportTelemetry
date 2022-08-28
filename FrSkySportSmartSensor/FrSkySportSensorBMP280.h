#ifndef _FRSKY_SPORT_SENSOR_BMP280_H_
#define _FRSKY_SPORT_SENSOR_BMP280_H_

#include "FrSkySportSensor.h"

#define BMP280_DEFAULT_ID ID1
#define BMP280_DATA_COUNT 3       // temperature, relative altitude, vertical speed
#define BMP280_ALT_DATA_ID 0x0100 // relative altitude to the start altitude in m
#define BMP280_VSI_DATA_ID 0x0110 // vertical speed in m/s
#define BMP280_T_DATA_ID 0x0400   // temperature in C

#define BMP280_DATA_PERIOD 500

class FrSkySportSensorBMP280 : public FrSkySportSensor
{
  public:
    FrSkySportSensorBMP280(SensorId id = BMP280_DEFAULT_ID);
    void setup();
    void calibrate();
    virtual uint16_t send(FrSkySportSingleWireSerial &serial, uint8_t id, uint32_t now);
    virtual uint16_t decodeData(uint8_t id, uint16_t appId, uint32_t data);

  private:
    uint32_t pressureTime;         // next time when pressure data should be sent
    uint32_t temperatureTime;      // next time when temperature data should be sent
    uint32_t verticalSpeedTime;    // next time when vertical speed data should be sent
    uint32_t pressureReadingTime;  // time when the last preassure measurement was registered
    bool sensorInitialized;        // true if setting up the sensor was successful
    double temperature;            // reading from the sensor
    double pressure;               // reading from the sensor
    double verticalSpeed;          // calculated
    double relativeAltitude;       // calculated from the baseLinePressure and the current preassure reading
    double baseLinePressure;       // zero level pressure at bootup as reference
};

#endif
