#ifndef _PLOTTER_H_
#define _PLOTTER_H_

#include "HardwareAccelerationSensor.h"
#include "HardwareGyroSensor.h"
#include "HardwareTemperatureSensor.h"
#include "HardwareAirPressureSensor.h"

class Plotter
{
  public:
    void SetTemperatureSensors(HardwareTemperatureSensor *temperatureSensors[]);
    void SetAirPressureSensors(HardwareAirPressureSensor *airPressureSensors[]);
    void Loop();

  private:
    HardwareTemperatureSensor** temperatureSensors;
    HardwareAirPressureSensor** airPressureSensors;
};

#endif
