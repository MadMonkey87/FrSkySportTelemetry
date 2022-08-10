/*
  FrSky TAS-01 3 axis acceleration sensor class for Teensy 3.x/4.0/LC, ESP8266, ATmega2560 (Mega) and ATmega328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Philippe Wechsler 20220810
  Not for commercial use
*/

#ifndef _FRSKY_SPORT_SENSOR_ACC_H_
#define _FRSKY_SPORT_SENSOR_ACC_H_

#include "FrSkySportSensor.h"

#define ACC_DEFAULT_ID ID23
#define ACC_DATA_COUNT 3
#define ACC_X_DATA_ID 0x0700
#define ACC_Y_DATA_ID 0x0710
#define ACC_Z_DATA_ID 0x0720

#define ACC_X_DATA_PERIOD 100
#define ACC_Y_DATA_PERIOD 100
#define ACC_Z_DATA_PERIOD 100

class FrSkySportSensorAcc : public FrSkySportSensor
{
  public:
    FrSkySportSensorAcc(SensorId id = ACC_DEFAULT_ID);
    void setData(float x, float y, float z);
    void setDataX(float x);
    void setDataY(float y);
    void setDataZ(float z);
    virtual uint16_t send(FrSkySportSingleWireSerial& serial, uint8_t id, uint32_t now);
    virtual uint16_t decodeData(uint8_t id, uint16_t appId, uint32_t data);
    float getAccX();
    float getAccY();
    float getAccZ();

  private:
    uint32_t xData;
    uint32_t yData;
    uint32_t zData;
    uint32_t accXTime;
    uint32_t accYTime;
    uint32_t accZTime;
    float x;
    float y;
    float z;
};

#endif
