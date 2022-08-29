#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_LSM303DLH_Mag.h>

#include "FrSkySportSensorLSM303Magnet.h"

Adafruit_LSM303DLH_Mag_Unified sensor = Adafruit_LSM303DLH_Mag_Unified(0);

FrSkySportSensorLSM303Magnet::FrSkySportSensorLSM303Magnet(SensorId id) : FrSkySportSensorOrientation(id) {}

void FrSkySportSensorLSM303Magnet::setup()
{
  Serial.println("Initialize LSM303 Magnetometer...");

  sensor.enableAutoRange(true);
  sensorInitialized = sensor.begin();

  if (sensorInitialized)
  {
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

    FrSkySportSensorOrientation::setup();

    Serial.println("done!\n");
  }
  else
  {
    Serial.println("failed!\n");
  }
}

void FrSkySportSensorLSM303Magnet::readSensorData() {
  sensors_event_t event;
  sensor.getEvent(&event);
  magnetometerX = event.magnetic.x;
  magnetometerY = event.magnetic.y;
  magnetometerZ = event.magnetic.z;
}

uint16_t FrSkySportSensorLSM303Magnet::getSampleRate() {
  return 100;
}
