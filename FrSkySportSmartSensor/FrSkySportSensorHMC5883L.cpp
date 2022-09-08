#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "FrSkySportSensorHMC5883L.h"

FrSkySportSensorHMC5883L::FrSkySportSensorHMC5883L()
{
  sensor = Adafruit_HMC5883_Unified(0);
}

bool FrSkySportSensorHMC5883L::Setup()
{
  if (!sensor.begin())
  {
    return false;
  }

  this->Ready = true;
  return true;
}

void FrSkySportSensorHMC5883L::UpdateSensorData()
{
  if (!this->Ready) {
    return;
  }

  sensors_event_t event;
  sensor.getEvent(&event);
  MagneticX = event.magnetic.x;
  MagneticY = event.magnetic.y;
  MagneticZ = event.magnetic.z;
}

char* FrSkySportSensorHMC5883L::GetName() {
  return "HMC5883L";
}
