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

  /*sensor_t sensorDetails;
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
  Serial.println(" uT");*/

  /*UpdateSensorData();
  Serial.print(" - x: ");
  Serial.print(MagneticX);
  Serial.print(" uT , y: ");
  Serial.print(MagneticY);
  Serial.print(" uT , z: ");
  Serial.print(MagneticZ);
  Serial.println(" uT");*/

  this->Ready = true;
  return true;
}

void FrSkySportSensorHMC5883L::UpdateSensorData()
{
  if(!this->Ready){
    return;
  }

  sensors_event_t event;
  sensor.getEvent(&event);
  MagneticX = event.magnetic.x;
  MagneticY = event.magnetic.y;
  MagneticZ = event.magnetic.z;
}

char* FrSkySportSensorHMC5883L::GetName(){
  return "HMC5883L";
}
