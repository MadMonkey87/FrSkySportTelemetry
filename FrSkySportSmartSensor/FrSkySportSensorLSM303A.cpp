#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "FrSkySportSensorLSM303A.h"

FrSkySportSensorLSM303A::FrSkySportSensorLSM303A()
{
  sensor = Adafruit_LSM303_Accel_Unified(0);
}

bool FrSkySportSensorLSM303A::Setup()
{
  if (!sensor.begin())
  {
    return false;
  }

  /*sensor_t sensorDetails;
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
  Serial.println(" m/s^2");*/

  this->Ready = true;
  return true;
}

void FrSkySportSensorLSM303A::UpdateSensorData()
{
  if(!this->Ready){
    return;
  }

  sensors_event_t event;
  sensor.getEvent(&event);
  AccelerationX = event.acceleration.x;
  AccelerationY = event.acceleration.y;
  AccelerationZ = event.acceleration.z;
}

char* FrSkySportSensorLSM303A::GetName(){
  return "LSM303A";
}
