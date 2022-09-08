#ifndef _FRSKY_SPORT_SENSOR_BMP280_H_
#define _FRSKY_SPORT_SENSOR_BMP280_H_

#include <Adafruit_BMP280.h>

#include "HardwareTemperatureSensor.h"
#include "HardwareAirPressureSensor.h"

class FrSkySportSensorBMP280 : public HardwareAirPressureSensor, public HardwareTemperatureSensor
{
  public:
    virtual bool Setup();
    virtual void UpdateSensorData();
    bool Ready;
    virtual char* GetName();

  private:
    Adafruit_BMP280 sensor;
    double baseAirPressure; //in Pa
};

#endif
