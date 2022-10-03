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
#include "SBusSensor.h"
#include "FrSkySportSensorBMP180.h"
#include "FrSkySportSensorBMP280.h"
#include "FrSkySportSensorLSM303M.h"
#include "FrSkySportSensorLSM303A.h"
#include "FrSkySportSensorMPU6050.h"
#include "FrSkySportSensorLSM6DS3.h"
#include "FrSkySportSensorHMC5883L.h"
#include "FrSkySportSensorTeensyOnBoard.h"
#include "FrSkySportSensorOrientation.h"
#include "FrSkySportSingleWireSerial.h"
#include "FrSkySportTelemetry.h"
#include "Plotter.h"
#if !defined(TEENSY_HW)
#include "SoftwareSerial.h"
#else
#include <InternalTemperature.h>
#endif

I2CScanner i2cScanner;
SBusSensor sbus;
FrSkySportSensorBMP180 bmp180;
FrSkySportSensorBMP280 bmp280;
FrSkySportSensorLSM303M lsm303m;
FrSkySportSensorLSM303A lsm303a;
FrSkySportSensorMPU6050 mpu6050;
FrSkySportSensorLSM6DS3 lsm6ds3;
FrSkySportSensorHMC5883L hmc5883l;
FrSkySportSensorTeensyOnBoard teensyOnBoard;
FrSkySportSensorOrientation orientationSensor;
Plotter plotter;
#ifdef POLLING_ENABLED
#include "FrSkySportPollingDynamic.h"
FrSkySportTelemetry telemetry(new FrSkySportPollingDynamic()); // Create telemetry object with dynamic (FrSky-like) polling
#else
FrSkySportTelemetry telemetry; // Create telemetry object without polling
#endif

#if defined(DEBUG)
unsigned long lastLoopTime = millis();
#endif

HardwareTemperatureSensor* temperatureSensors[] =
{
  &bmp180, &bmp280, &teensyOnBoard
};

HardwareAirPressureSensor* airpressureSensors[] =
{
  &bmp180, &bmp280
};

HardwareAccelerationSensor* accelerationSensors[] =
{
  &mpu6050, &lsm6ds3, &lsm303a
};

HardwareGyroSensor* gyroSensors[] =
{
  &mpu6050, &lsm6ds3
};

HardwareMagneticSensor* magneticSensors[] =
{
  &hmc5883l, &lsm303m
};

void setup()
{
  delay(1000);

  Serial.println("Booting SmartPort multi sensor");
  Serial.println(" - v0.1 Alpha");
  Serial.print(" - Compile time: "); Serial.println(__TIMESTAMP__);

#if defined(DEBUG)
  Serial.println(" - Debug: yes");
#else
  Serial.println(" - Debug: no");
#endif
#if defined(TEENSY_HW)
  Serial.println(" - Running on teensy: yes");
  Serial.print(" - unique Teensy Id: "); Serial.println(InternalTemperature.getUniqueID(), HEX);
  Serial.print(" - CPU voltage: "); Serial.print(InternalTemperature.getVTemp25()); Serial.println(" V");
#else
  Serial.println(" - Running on teensy: no\n");
#endif
#if defined(F_CPU)
  Serial.print(" - Target clockspeed: "); Serial.print(F_CPU / 1000000); Serial.println(" Mhz");
#endif
#if defined(F_CPU_ACTUAL)
  Serial.print(" - Actual clockspeed: "); Serial.print(F_CPU_ACTUAL / 1000000); Serial.println(" Mhz");
#endif
#if defined(F_BUS_ACTUAL)
  Serial.print(" - Actual bus speed: "); Serial.print(F_BUS_ACTUAL / 1000000); Serial.println(" Mhz");
#endif
  Serial.println("");

  i2cScanner.scan();

  sbus.Setup();

  Serial.print("Initialize Smart Port...\n");
  // Configure the telemetry serial port and sensors (remember to use & to specify a pointer to sensor)
#if defined(TEENSY_HW)
  telemetry.begin(FrSkySportSingleWireSerial::SERIAL_3, &orientationSensor);
#else
  telemetry.begin(FrSkySportSingleWireSerial::SOFT_SERIAL_PIN_12, &orientationSensor);
#endif
  Serial.println("done!\n");

  void* hardwareSensors[] = {
    &bmp180,
    &bmp280,
    &lsm303m,
    &lsm303a,
    &mpu6050,
    &lsm6ds3,
    &hmc5883l,
    &teensyOnBoard
  };

  unsigned int hardwareTemperatureSensors = 0;
  unsigned int hardwareAccelerometerSensors = 0;
  unsigned int hardwareGyroSensors = 0;
  unsigned int hardwareMagneticSensors = 0;
  unsigned int hardwareAirPressureSensors = 0;
  unsigned int unavailableHardwareSensors = 0;

  Serial.print("Initializing "); Serial.print(sizeof(hardwareSensors) / sizeof(hardwareSensors[0])); Serial.println(" hardware sensors...\n");
  for (unsigned int i = 0; i < sizeof(hardwareSensors) / sizeof(hardwareSensors[0]); ++i) {
    HardwareSensor* hardwareSensor = (HardwareSensor*)hardwareSensors[i];
    Serial.print("Initializing "); Serial.print(hardwareSensor->GetName()); Serial.println("...");
    if (hardwareSensor->Setup()) {
      hardwareSensor->UpdateSensorData();

      if (hardwareSensor->IsHardwareTemperatureSensor())
      {
        hardwareTemperatureSensors++;
      }

      if (hardwareSensor->IsHardwareAccelerationSensor())
      {
        hardwareAccelerometerSensors++;
      }

      if (hardwareSensor->IsHardwareGyroSensor())
      {
        hardwareGyroSensors++;
      }

      if (hardwareSensor->IsHardwareMagneticSensor())
      {
        hardwareMagneticSensors++;
      }

      if (hardwareSensor->IsHardwareAirPressureSensor())
      {
        hardwareAirPressureSensors++;
      }

      Serial.println("successfull!\n");
    } else {
      unavailableHardwareSensors++;
      Serial.println("failed!\n");
    }
  }

  HardwareAccelerationSensor *accelerationSensor = &mpu6050;
  HardwareGyroSensor *gyroSensor = &mpu6050;
  HardwareMagneticSensor *magneticSensor = &lsm303m;
  orientationSensor.Setup(accelerationSensor, gyroSensor, magneticSensor);

  plotter.SetTemperatureSensors(temperatureSensors, sizeof(temperatureSensors) / sizeof(HardwareTemperatureSensor*));
  plotter.SetAirPressureSensors(airpressureSensors, sizeof(airpressureSensors) / sizeof(HardwareAirPressureSensor*));
  plotter.SetAccelerationSensors(accelerationSensors, sizeof(accelerationSensors) / sizeof(HardwareAccelerationSensor*));
  plotter.SetGyroSensors(gyroSensors, sizeof(gyroSensors) / sizeof(HardwareGyroSensor*));
  plotter.SetMagneticSensors(magneticSensors, sizeof(magneticSensors) / sizeof(HardwareMagneticSensor*));
  plotter.PrintDetails();

  Serial.println();
  Serial.println("Setup completed! Start operating...\n");
}

bool blink = false;

void loop()
{





  if (blink) {
    blink = false;
    digitalWrite(13, HIGH);
  } else {
    digitalWrite(13, LOW);
    blink = true;
  }

#if defined(DEBUG)
  unsigned long now = millis();
  Serial.print("loop time (ms): ");
  Serial.println(now - lastLoopTime);
  lastLoopTime = now;
#endif






  //sbusListener.update();

#ifdef POLLING_ENABLED
  // Set receiver data to be sent in case the polling is enabled (so no actual receiver is used)
  telemetry.setData(90,   // RSSI value (0-100, 0 = no telemetry, 100 = full signal)
                    4.9); // RxBatt (voltage supplied to the receiver) value in volts (0.0-13.2)
#endif

  // Send the telemetry data, note that the data will only be sent for sensors
  // that are being polled at given moment
  telemetry.send();





  sbus.UpdateSensorData();
  bmp180.UpdateSensorData();
  bmp280.UpdateSensorData();
  lsm303m.UpdateSensorData();
  lsm303a.UpdateSensorData();
  mpu6050.UpdateSensorData();
  hmc5883l.UpdateSensorData();
  lsm6ds3.UpdateSensorData();
  teensyOnBoard.UpdateSensorData();


  orientationSensor.readAndCalculate();





  /*for (int i = 0; i < sizeof(hardwareSensors) / sizeof(hardwareSensors[0]); ++i) {
    HardwareSensor* hardwareSensor = (HardwareSensor*)hardwareSensors[i];
    hardwareSensor->UpdateSensorData();
    }*/
  //Serial.print("bpm180:"); Serial.print(bmp180.RelativeAltitude); Serial.print(" bpm280:"); Serial.println(bmp280.RelativeAltitude);
  //Serial.print("mpu6050:");Serial.print(mpu6050.Temperature);Serial.print(" bpm180:");Serial.print(bmp180.Temperature);Serial.print(" bpm280:");Serial.println(bmp280.Temperature);

  //plotter.PrintDetails();

  //plotter.PlotTemperatures();
  //plotter.PlotAirPressures();
  //plotter.PlotRealativeAltitudes();
  //plotter.PlotAccelerationValues();
  //plotter.PlotGForceValues();
  //plotter.PlotGyroValues();
  //plotter.PlotMagneticValues();
}
