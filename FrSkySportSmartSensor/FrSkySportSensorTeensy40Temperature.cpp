#include "FrSkySportSensorTeensy40Temperature.h"
#include <Wire.h>

//extern float tempmonGetTemp(void);

bool FrSkySportSensorTeensy40Temperature::IsReady() {
  return Ready;
}

bool FrSkySportSensorTeensy40Temperature::Setup() {
  Ready = true;
  return true;
}

void FrSkySportSensorTeensy40Temperature::UpdateSensorData() {
  if (!Ready) {
    return;
  }
  Temperature = 123;//tempmonGetTemp();
}

char* FrSkySportSensorTeensy40Temperature::GetName() {
  return "Teensy 4.0 internal temperature sensor";
}
