#ifndef _SENSOR_MPU6050_H_
#define _SENSOR_MPU6050_H_

#include <Adafruit_MPU6050.h>

#include "HardwareAccelerationSensor.h"
#include "HardwareGyroSensor.h"
#include "HardwareTemperatureSensor.h"

class SensorMPU6050 : public HardwareAccelerationSensor, public HardwareGyroSensor, public HardwareTemperatureSensor
{
  public:
    virtual bool Setup();
    virtual void UpdateSensorData();
    virtual char* GetName();
    virtual bool IsReady();

  private:
    Adafruit_MPU6050 mpu;
    Adafruit_Sensor *temperatureSensor, *accelerationSensor, *gyroSensor;
    bool Ready;

};

#endif
