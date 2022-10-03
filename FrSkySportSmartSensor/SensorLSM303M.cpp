#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "SensorLSM303M.h"

bool SensorLSM303M::IsReady() {
  return Ready;
}

SensorLSM303M::SensorLSM303M()
{
  sensor = Adafruit_LSM303_Mag_Unified(0);
}

bool SensorLSM303M::Setup()
{
  if (!sensor.begin())
  {
    return false;
  }

  sensor_t sensorDetails;
  sensor.getSensor(&sensorDetails);
  Serial.print(" - Sensor: ");
  Serial.println(sensorDetails.name);
  Serial.print(" - Driver Ver: ");
  Serial.println(sensorDetails.version);
  Serial.print(" - Max Value: ");
  Serial.print(sensorDetails.max_value);
  Serial.println(" uT");
  Serial.print(" - Min Value: ");
  Serial.print(sensorDetails.min_value);
  Serial.println(" uT");
  Serial.print(" - Resolution: ");
  Serial.print(sensorDetails.resolution);
  Serial.println(" uT");

  Ready = true;
  return true;
}

void SensorLSM303M::UpdateSensorData()
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

char* SensorLSM303M::GetName() {
  return "LSM303M";
}
