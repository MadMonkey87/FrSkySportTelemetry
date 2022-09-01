#ifndef _HARDWARE_MAGNETIC_SENSOR_H_
#define _HARDWARE_MAGNETIC_SENSOR_H_

class HardwareMagneticSensor
{
  public:
    virtual bool Setup();
    virtual void UpdateSensorData();
    double MagneticX, MagneticY, MagneticZ; 
};

#endif
