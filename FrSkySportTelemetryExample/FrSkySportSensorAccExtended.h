/*
  FrSky TAS-01 3 axis acceleration sensor class for Teensy 3.x/4.0/LC, ESP8266, ATmega2560 (Mega) and ATmega328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Philippe Wechsler 20220810
  Not for commercial use
*/

#ifndef _FRSKY_SPORT_SENSOR_ACCEXT_H_
#define _FRSKY_SPORT_SENSOR_ACCEXT_H_

#include "FrSkySportSensorAcc.h"

#define ACCEXT_DEFAULT_ID ID24

class FrSkySportSensorAccExtended : public FrSkySportSensorAcc
{
public:
  FrSkySportSensorAccExtended(SensorId id = ACCEXT_DEFAULT_ID);
};

#endif
