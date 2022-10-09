#ifndef _PLOTTER_H_
#define _PLOTTER_H_

#include "HardwareAccelerationSensor.h"
#include "HardwareGyroSensor.h"
#include "HardwareMagneticSensor.h"
#include "HardwareTemperatureSensor.h"
#include "HardwareAirPressureSensor.h"
#include "Arduino.h"

class Plotter
{
  public:
    void Setup();
    void SetTemperatureSensors(HardwareTemperatureSensor *temperatureSensors[], unsigned int count);
    void SetAirPressureSensors(HardwareAirPressureSensor *airPressureSensors[], unsigned int count);
    void SetAccelerationSensors(HardwareAccelerationSensor *accelerationSensors[], unsigned int count);
    void SetGyroSensors(HardwareGyroSensor *gyroSensors[], unsigned int count);
    void SetMagneticSensors(HardwareMagneticSensor *magneticSensors[], unsigned int count);
    void PrintDetails();
    void PlotTemperatures();
    void PlotAirPressures();
    void PlotRealativeAltitudes();
    void PlotAccelerationValues();
    void PlotGForceValues();
    void PlotGyroValues();
    void PlotMagneticValues();
    void Log();

  private:
    HardwareTemperatureSensor** temperatureSensors;
    unsigned int temperatureSensorsCount;

    HardwareAirPressureSensor** airPressureSensors;
    unsigned int airPressureSensorsCount;

    HardwareAccelerationSensor** accelerationSensors;
    unsigned int accelerationSensorsCount;

    HardwareGyroSensor** gyroSensors;
    unsigned int gyroSensorsCount;

    HardwareMagneticSensor** magneticSensors;
    unsigned int magneticSensorsCount;

    uint32_t logTime;
};

#endif
