#ifndef _SENSOR_BMP280_H_
#define _SENSOR_BMP280_H_

#include <Adafruit_BMP280.h>

#include "HardwareTemperatureSensor.h"
#include "HardwareAirPressureSensor.h"

class SensorBMP280 : public HardwareAirPressureSensor, public HardwareTemperatureSensor
{
  public:
    virtual bool Setup();
    virtual void UpdateSensorData();
    virtual char* GetName();
    virtual bool IsReady();

  private:
    Adafruit_BMP280 sensor;
    double baseAirPressure; //in Pa
    bool Ready;
};

#endif
