#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "SensorHMC5883L.h"

bool SensorHMC5883L::IsReady(){
  return Ready;
}

SensorHMC5883L::SensorHMC5883L()
{
  sensor = Adafruit_HMC5883_Unified(0);
}

bool SensorHMC5883L::Setup()
{
  if (!sensor.begin())
  {
    return false;
  }

  Ready = true;
  return true;
}

void SensorHMC5883L::UpdateSensorData()
{
  if (!Ready) {
    return;
  }

  sensors_event_t event;
  sensor.getEvent(&event);
  MagneticX = event.magnetic.x;
  MagneticY = event.magnetic.y;
  MagneticZ = event.magnetic.z;
}

char* SensorHMC5883L::GetName() {
  return "HMC5883L";
}
