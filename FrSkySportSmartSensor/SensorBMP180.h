#ifndef _SENSOR_BMP180_H_
#define _SENSOR_BMP180_H_

#include <Adafruit_BMP085.h>

#include "HardwareTemperatureSensor.h"
#include "HardwareAirPressureSensor.h"

class SensorBMP180 : public HardwareAirPressureSensor, public HardwareTemperatureSensor
{
  public:
    virtual bool Setup();
    virtual void UpdateSensorData();
    virtual char* GetName();
    virtual bool IsReady();

  private:
    Adafruit_BMP085 sensor;
    double baseAirPressure; //in Pa
    bool Ready;
};

#endif
