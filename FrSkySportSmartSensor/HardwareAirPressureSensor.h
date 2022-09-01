#ifndef _HARDWARE_AIR_PRESSURE_SENSOR_H_
#define _HARDWARE_AIR_PRESSURE_SENSOR_H_

class HardwareAirPressureSensor
{
  public:
    virtual bool Setup();
    virtual void UpdateSensorData();
    double AirPressure; // in hPa
    double RelativeAltitude; //in m
};

#endif
