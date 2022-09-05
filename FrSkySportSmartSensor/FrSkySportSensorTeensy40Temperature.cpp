#include "FrSkySportSensorTeensy40Temperature.h"
#include <Wire.h>

//extern float tempmonGetTemp(void);

bool FrSkySportSensorTeensy40Temperature::Setup(){
  this->Ready = true;
  return true;
}

void FrSkySportSensorTeensy40Temperature::UpdateSensorData(){
  if(!this->Ready){
    return;
  }
  Temperature = 123;//tempmonGetTemp();
}

char* FrSkySportSensorTeensy40Temperature::GetName(){
  return "Teensy 4.0 internal temperature sensor";
}
