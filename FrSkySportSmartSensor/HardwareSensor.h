#ifndef _HARDWARE_SENSOR_H_
#define _HARDWARE_SENSOR_H_

class HardwareSensor
{
  public:
    virtual bool Setup();
    virtual void UpdateSensorData(); 
    bool Ready;
};

#endif
