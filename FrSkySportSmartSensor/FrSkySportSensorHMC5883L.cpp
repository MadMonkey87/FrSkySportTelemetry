#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_HMC5883_U.h>

#include "FrSkySportSensorHMC5883L.h"

Adafruit_HMC5883_Unified sensorHCM5883L = Adafruit_HMC5883_Unified(0);

FrSkySportSensorHMC5883L::FrSkySportSensorHMC5883L(SensorId id) : FrSkySportSensorOrientation(id) {}

void FrSkySportSensorHMC5883L::setup()
{
  Serial.println("Initialize HMC5883L...");

  sensorInitialized = sensorHCM5883L.begin(); 

  if (sensorInitialized)
  {
    sensor_t sensorDetails;
    sensorHCM5883L.getSensor(&sensorDetails);
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

    FrSkySportSensorOrientation::setup();
 
    Serial.println("done!\n");
  }
  else
  {
    Serial.println("failed!\n");
  }
}

void FrSkySportSensorHMC5883L::readSensorData() {
  sensors_event_t event;
  sensorHCM5883L.getEvent(&event);
  magnetometerX = event.magnetic.x;
  magnetometerY = event.magnetic.y;
  magnetometerZ = event.magnetic.z;
}

uint16_t FrSkySportSensorHMC5883L::getSampleRate() {
  return 160;
}
