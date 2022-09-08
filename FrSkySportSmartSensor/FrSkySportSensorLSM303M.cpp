#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "FrSkySportSensorLSM303M.h"

FrSkySportSensorLSM303M::FrSkySportSensorLSM303M()
{
  sensor = Adafruit_LSM303_Mag_Unified(0);
}

bool FrSkySportSensorLSM303M::Setup()
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

  this->Ready = true;
  return true;
}

void FrSkySportSensorLSM303M::UpdateSensorData()
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

char* FrSkySportSensorLSM303M::GetName() {
  return "LSM303M";
}
