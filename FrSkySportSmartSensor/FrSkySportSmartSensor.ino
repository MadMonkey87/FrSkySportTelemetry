/*
  FrSky S-Port Telemetry library example
  (c) Pawelsky 202000503
  Not for commercial use

  Note that you need Teensy 3.x/4.0/LC, ESP8266, ATmega2560 (Mega) or ATmega328P based (e.g. Pro Mini, Nano, Uno) board and FrSkySportTelemetry library for this example to work
*/

// #define DEBUG

// Uncomment the #define below to enable internal polling of data.
// Use only when there is no device in the S.Port chain (e.g. S.Port capable FrSky receiver) that normally polls the data.
//#define POLLING_ENABLED

#include "I2CScanner.h"
#include "SBusListener.h"
#include "FrSkySportSensorBMP180.h"
#include "FrSkySportSensorBMP280.h"
#include "FrSkySportSingleWireSerial.h"
#include "FrSkySportTelemetry.h"
#if !defined(TEENSY_HW)
#include "SoftwareSerial.h"
#endif

I2CScanner i2cScanner;
SBusListener sbusListener;
FrSkySportSensorBMP180 bmp180;
FrSkySportSensorBMP280 bmp280;
#ifdef POLLING_ENABLED
#include "FrSkySportPollingDynamic.h"
FrSkySportTelemetry telemetry(new FrSkySportPollingDynamic()); // Create telemetry object with dynamic (FrSky-like) polling
#else
FrSkySportTelemetry telemetry; // Create telemetry object without polling
#endif

#if defined(DEBUG)
unsigned long lastLoopTime = millis();
#endif

void setup()
{
  delay(1000);

  Serial.print("Compile time: ");
  Serial.println(__DATE__ " " __TIME__);
  Serial.print("File: ");
  Serial.println(__FILE__);

#if defined(DEBUG)
  Serial.println("Debug: yes");
#else
  Serial.println("Debug: no");
#endif
#if defined(TEENSY_HW)
  Serial.println("Running on teensy: yes");
#else
  Serial.println("Running on teensy: no\n");
#endif

  Serial.println("\nBooting SmartPort multi sensor\n");

  i2cScanner.scan();
  sbusListener.setup();

  Serial.print("Initialize Smart Port...\n");
  // Configure the telemetry serial port and sensors (remember to use & to specify a pointer to sensor)
#if defined(TEENSY_HW)
  telemetry.begin(FrSkySportSingleWireSerial::SERIAL_3, &bmp180, &bmp280);
#else
  telemetry.begin(FrSkySportSingleWireSerial::SOFT_SERIAL_PIN_12, &bmp180, &bmp280);
#endif
  Serial.println("done!\n");

  bmp180.setup();
  bmp280.setup();
}

void loop()
{
#if defined(DEBUG)
  unsigned long now = millis();
  Serial.print("loop time (ms): ");
  Serial.println(now - lastLoopTime);
  lastLoopTime = now;
#endif

  sbusListener.update();

#ifdef POLLING_ENABLED
  // Set receiver data to be sent in case the polling is enabled (so no actual receiver is used)
  telemetry.setData(90,   // RSSI value (0-100, 0 = no telemetry, 100 = full signal)
                    4.9); // RxBatt (voltage supplied to the receiver) value in volts (0.0-13.2)
#endif

  // Send the telemetry data, note that the data will only be sent for sensors
  // that are being polled at given moment
  telemetry.send();
}
