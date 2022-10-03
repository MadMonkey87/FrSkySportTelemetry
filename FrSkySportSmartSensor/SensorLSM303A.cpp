#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "SensorLSM303A.h"

SensorLSM303A::SensorLSM303A()
{
  sensor = Adafruit_LSM303_Accel_Unified(0);
}

bool SensorLSM303A::IsReady() {
  return Ready;
}

bool SensorLSM303A::Setup()
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
  Serial.println(" m/s^2");
  Serial.print(" - Min Value: ");
  Serial.print(sensorDetails.min_value);
  Serial.println(" m/s^2");
  Serial.print(" - Resolution: ");
  Serial.print(sensorDetails.resolution);
  Serial.println(" m/s^2");

  Ready = true;
  return true;
}

void SensorLSM303A::UpdateSensorData()
{
  if (!Ready) {
    return;
  }

  sensors_event_t event;
  sensor.getEvent(&event);
  AccelerationX = event.acceleration.x;
  AccelerationY = event.acceleration.y;
  AccelerationZ = event.acceleration.z;
}

char* SensorLSM303A::GetName() {
  return "LSM303A";
}
