#ifndef _HARDWARE_TEMPERATURE_SENSOR_H_
#define _HARDWARE_TEMPERATURE_SENSOR_H_

class HardwareTemperatureSensor
{
  public:
    virtual bool Setup();
    virtual void UpdateSensorData();
    double Temperature; // in C
};

#endif
