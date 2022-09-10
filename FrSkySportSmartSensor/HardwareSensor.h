#ifndef _HARDWARE_SENSOR_H_
#define _HARDWARE_SENSOR_H_

class HardwareSensor
{
  public:
    virtual bool Setup();
    virtual void UpdateSensorData();
    virtual bool IsReady();

    virtual bool IsHardwareTemperatureSensor();
    virtual bool IsHardwareAccelerationSensor();
    virtual bool IsHardwareGyroSensor();
    virtual bool IsHardwareMagneticSensor();
    virtual bool IsHardwareAirPressureSensor();
    virtual char* GetName();
};

#endif
