#ifndef _SBUS_Listener_H_
#define _SBUS_Listener_H_

#define SBUS_DATA_PERIOD 250

#include <stdint.h>

class SBusListener
{
public:
  SBusListener();
  void setup();
  void update();

 private:
  uint32_t sbusTime;
};

#endif
