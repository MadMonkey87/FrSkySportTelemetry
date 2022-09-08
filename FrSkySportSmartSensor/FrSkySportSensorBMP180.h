#ifndef _FRSKY_SPORT_SENSOR_BMP180_H_
#define _FRSKY_SPORT_SENSOR_BMP180_H_

#include <Adafruit_BMP085.h>

#include "HardwareTemperatureSensor.h"
#include "HardwareAirPressureSensor.h"

class FrSkySportSensorBMP180 : public HardwareAirPressureSensor, public HardwareTemperatureSensor
{
  public:
    virtual bool Setup();
    virtual void UpdateSensorData();
    bool Ready;
    virtual char* GetName();

  private:
    Adafruit_BMP085 sensor;
    double baseAirPressure; //in Pa
};

#endif
