#include "FrSkySportSensorTeensy40Temperature.h"
#include <Wire.h>

extern float tempmonGetTemp(void);

bool FrSkySportSensorTeensy40Temperature::Setup(){
  Serial.println("Initialize Teensy 4.0 internal temperature sensor...");

  Serial.print(" - Temperature: ");Serial.print(tempmonGetTemp());Serial.println(" C°");
    
  this->Ready = true;
  Serial.println("done!\n");
  return true;
}

void FrSkySportSensorTeensy40Temperature::UpdateSensorData(){
  if(!this->Ready){
    return;
  }
  Temperature = tempmonGetTemp();
}
