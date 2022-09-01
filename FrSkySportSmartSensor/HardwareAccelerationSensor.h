#ifndef _HARDWARE_ACCELERATION_SENSOR_H_
#define _HARDWARE_ACCELERATION_SENSOR_H_

class HardwareAccelerationSensor
{
  public:
    virtual bool Setup();
    virtual void UpdateSensorData();
    double AccelerationX, AccelerationY, AccelerationZ; 
};

#endif
