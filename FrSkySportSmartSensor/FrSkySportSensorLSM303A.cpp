#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "FrSkySportSensorLSM303A.h"

FrSkySportSensorLSM303A::FrSkySportSensorLSM303A()
{
  sensor = Adafruit_LSM303_Accel_Unified(0);
}

bool FrSkySportSensorLSM303A::Setup()
{
  Serial.println("Initialize LSM303A...");

  if (!sensor.begin())
  {
    Serial.println("failed!\n");
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

  Serial.println("done!\n");

  return true;
}

void FrSkySportSensorLSM303A::UpdateSensorData()
{
  sensors_event_t event;
  sensor.getEvent(&event);
  AccelerationX = event.acceleration.x;
  AccelerationY = event.acceleration.y;
  AccelerationZ = event.acceleration.z;
}
