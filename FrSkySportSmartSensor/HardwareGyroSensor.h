#ifndef _HARDWARE_GYRO_SENSOR_H_
#define _HARDWARE_GYRO_SENSOR_H_

class HardwareGyroSensor
{
  public:
    virtual bool Setup();
    virtual void UpdateSensorData();
    double GyroX, GyroY, GyroZ; 
};

#endif
