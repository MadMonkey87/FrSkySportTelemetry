#include "Plotter.h"
#include <Wire.h>

void Plotter::SetTemperatureSensors(HardwareTemperatureSensor *temperatureSensors[]) {
  this->temperatureSensors = temperatureSensors;
}

void Plotter::SetAirPressureSensors(HardwareAirPressureSensor *airPressureSensors[]) {
  this->airPressureSensors = airPressureSensors;
}

void Plotter::Loop() {
  //Serial.println(this->temperatureSensors->Temperature);

  for (int i = 0; i < 2; i++) {
    Serial.print(temperatureSensors[i]->GetName());
    Serial.print(": ");
    Serial.println(temperatureSensors[i]->Temperature);
  }
}
