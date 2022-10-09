#include "SBusSensor.h"
#include "sbus.h"

/* SbusRx object on Serial1 */
bfs::SbusRx sbus_rx(&Serial1);
/* Array for storing SBUS data */
std::array<int16_t, bfs::SbusRx::NUM_CH()> sbus_data;

bool SBusSensor::Setup()
{
  Serial.println("Initialize SBUS...");

  sbus_rx.Begin();

  for (int i = 0; i < 100; i++) {
    if (sbus_rx.Read()) {
      sbus_data = sbus_rx.ch();
      Serial.print(" - channels: "); Serial.println(bfs::SbusRx::NUM_CH());
      for (int8_t i = 0; i < bfs::SbusRx::NUM_CH(); i++)
      {
        Serial.print(" - channel ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(sbus_data[i]);
      }
      Serial.print(" - lost-frame: ");
      Serial.println(sbus_rx.lost_frame());
      Serial.print(" - failsafe: ");
      Serial.println(sbus_rx.failsafe());

      Ready = true;
      return  true;
    } else if (i == 99) {
      Serial.println(" - not connected!");
    }
    delay(10);
  }
  return false;
}

void SBusSensor::UpdateSensorData()
{
  uint32_t now = millis();
  if (now >= sbusTime)
  {
    if (sbus_rx.Read())
    {
      sbus_data = sbus_rx.ch();

      /*for (int8_t i = 0; i < bfs::SbusRx::NUM_CH(); i++)
        {
        Serial.print(sbus_data[i]);
        Serial.print("\t");
        }
        Serial.print(sbus_rx.lost_frame());
        Serial.print("\t");
        Serial.println(sbus_rx.failsafe());*/

      sbusTime = now + SBUS_DATA_PERIOD;
    }
  }
}

char* SBusSensor::GetName() {
  return "SBus";
}

bool SBusSensor::IsReady() {
  return Ready;
}
