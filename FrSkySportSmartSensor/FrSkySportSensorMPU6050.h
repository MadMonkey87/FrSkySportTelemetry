#ifndef _FRSKY_SPORT_SENSOR_MPU6050_H_
#define _FRSKY_SPORT_SENSOR_MPU6050_H_

#include <Adafruit_MPU6050.h>

#include "HardwareAccelerationSensor.h"
#include "HardwareGyroSensor.h"
#include "HardwareTemperatureSensor.h"

class FrSkySportSensorMPU6050 : public HardwareAccelerationSensor, public HardwareGyroSensor, public HardwareTemperatureSensor
{
  public:
    virtual bool Setup();
    virtual void UpdateSensorData();
    bool Ready;
    virtual char* GetName();

  private:
    Adafruit_MPU6050 mpu;
    Adafruit_Sensor *temperatureSensor, *accelerationSensor, *gyroSensor;

};

#endif
