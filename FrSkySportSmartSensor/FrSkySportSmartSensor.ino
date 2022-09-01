/*
  FrSky S-Port Telemetry library example
  (c) Pawelsky 202000503
  Not for commercial use

  Note that you need Teensy 3.x/4.0/LC, ESP8266, ATmega2560 (Mega) or ATmega328P based (e.g. Pro Mini, Nano, Uno) board and FrSkySportTelemetry library for this example to work
*/

//#define DEBUG

// Uncomment the #define below to enable internal polling of data.
// Use only when there is no device in the S.Port chain (e.g. S.Port capable FrSky receiver) that normally polls the data.
//#define POLLING_ENABLED

#include "I2CScanner.h"
#include "SBusListener.h"
#include "FrSkySportSensorBMP180.h"
#include "FrSkySportSensorBMP280.h"
#include "FrSkySportSensorLSM303M.h"
#include "FrSkySportSensorLSM303A.h"
#include "FrSkySportSensorMPU6050.h"
#include "FrSkySportSensorLSM6DS3.h"
#include "FrSkySportSensorHMC5883L.h"
#include "FrSkySportSingleWireSerial.h"
#include "FrSkySportTelemetry.h"
#if !defined(TEENSY_HW)
#include "SoftwareSerial.h"
#endif

I2CScanner i2cScanner;
SBusListener sbusListener;
FrSkySportSensorBMP180 bmp180;
FrSkySportSensorBMP280 bmp280;
FrSkySportSensorLSM303M lsm303m;
FrSkySportSensorLSM303A lsm303a;
FrSkySportSensorMPU6050 mpu6050;
FrSkySportSensorLSM6DS3 lsm6ds3;
FrSkySportSensorHMC5883L hmc5883l;
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
  Serial.println("Booting SmartPort multi sensor\n");
  Serial.println("v0.1 Alpha\n");
  Serial.print("Compile time: ");Serial.println(__TIMESTAMP__);

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

  i2cScanner.scan();

  sbusListener.setup();

  Serial.print("Initialize Smart Port...\n");
  // Configure the telemetry serial port and sensors (remember to use & to specify a pointer to sensor)
#if defined(TEENSY_HW)
  telemetry.begin(FrSkySportSingleWireSerial::SERIAL_3);
#else
  telemetry.begin(FrSkySportSingleWireSerial::SOFT_SERIAL_PIN_12);
#endif
  Serial.println("done!\n");

  bmp180.Setup();
  bmp280.Setup();
  lsm303m.Setup();
  lsm303a.Setup();
  mpu6050.Setup();
  lsm6ds3.Setup();
  hmc5883l.Setup();

  Serial.println("setup completed!\n");
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

  //mpu6050.loop();
  //lsm6ds3.loop();
  //lsm303magnetometer.loop();
  //hmc5883l.loop();

  bmp180.UpdateSensorData();
  bmp280.UpdateSensorData();
  lsm303m.UpdateSensorData();
  lsm303a.UpdateSensorData();
  mpu6050.UpdateSensorData();
  hmc5883l.UpdateSensorData();
  lsm6ds3.UpdateSensorData();
  //Serial.print("bpm180:");Serial.print(bmp180.RelativeAltitude);Serial.print(" bpm280:");Serial.println(bmp280.RelativeAltitude);
  //Serial.print("mpu6050:");Serial.print(mpu6050.Temperature);Serial.print("bpm180:");Serial.print(bmp180.Temperature);Serial.print(" bpm280:");Serial.println(bmp280.Temperature);
}
