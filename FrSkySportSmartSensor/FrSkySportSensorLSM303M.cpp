#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "FrSkySportSensorLSM303M.h"

FrSkySportSensorLSM303M::FrSkySportSensorLSM303M()
{
  sensor = Adafruit_LSM303_Mag_Unified(0);
}

bool FrSkySportSensorLSM303M::Setup()
{
  Serial.println("Initialize LSM303M...");

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
  Serial.println(" uT");
  Serial.print(" - Min Value: ");
  Serial.print(sensorDetails.min_value);
  Serial.println(" uT");
  Serial.print(" - Resolution: ");
  Serial.print(sensorDetails.resolution);
  Serial.println(" uT");

  UpdateSensorData();
  Serial.print(" - x: ");
  Serial.print(MagneticX);
  Serial.print(" uT , y: ");
  Serial.print(MagneticY);
  Serial.print(" uT , z: ");
  Serial.print(MagneticZ);
  Serial.println(" uT");

  Ready = true;
  Serial.println("done!\n");
  return true;
}

void FrSkySportSensorLSM303M::UpdateSensorData()
{
  if(!Ready){
    return;
  }

  sensors_event_t event;
  sensor.getEvent(&event);
  MagneticX = event.magnetic.x;
  MagneticY = event.magnetic.y;
  MagneticZ = event.magnetic.z;
}
