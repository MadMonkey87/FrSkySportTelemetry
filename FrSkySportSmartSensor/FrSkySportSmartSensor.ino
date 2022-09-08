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
#include "FrSkySportSensorTeensy40Temperature.h"
#include "FrSkySportSensorOrientation.h"
#include "FrSkySportSingleWireSerial.h"
#include "FrSkySportTelemetry.h"
#include "Plotter.h"
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
FrSkySportSensorTeensy40Temperature t40t;
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

  sbusListener.setup();

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
    &t40t
  };

  int hardwareTemperatureSensors = 0;
  int hardwareAccelerometerSensors = 0;
  int hardwareGyroSensors = 0;
  int hardwareMagneticSensors = 0;
  int hardwareAirPressureSensors = 0;
  int unavailableHardwareSensors = 0;

  Serial.print("Initializing "); Serial.print(sizeof(hardwareSensors) / sizeof(hardwareSensors[0])); Serial.println(" hardware sensors...\n");
  for (int i = 0; i < sizeof(hardwareSensors) / sizeof(hardwareSensors[0]); ++i) {
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

  Serial.println("Completed hardware sensors:");
  Serial.print(" - temperature sensors: "); Serial.println(hardwareTemperatureSensors);
  Serial.print(" - acceleration sensors: "); Serial.println(hardwareAccelerometerSensors);
  Serial.print(" - gyro sensors: "); Serial.println(hardwareGyroSensors);
  Serial.print(" - magnetic sensors: "); Serial.println(hardwareMagneticSensors);
  Serial.print(" - air pressure sensors: "); Serial.println(hardwareAirPressureSensors);
  Serial.print(" - unavailable sensors: "); Serial.println(unavailableHardwareSensors);
  Serial.println();

  Serial.println("Temperature values:");
  for (int i = 0; i < sizeof(hardwareSensors) / sizeof(hardwareSensors[0]); ++i) {
    HardwareSensor* hardwareSensor = (HardwareSensor*)hardwareSensors[i];
    if (hardwareSensor->IsHardwareTemperatureSensor()) {
      HardwareTemperatureSensor* hardwareTemperatureSensor = (HardwareTemperatureSensor*)hardwareSensors[i];
      Serial.print(" - ");
      Serial.print(hardwareSensor->GetName());
      Serial.print(": ");
      Serial.print(hardwareTemperatureSensor->Temperature);
      Serial.println(" C°");
    }
  }

  Serial.println("\nAccelerometer values:");
  for (int i = 0; i < sizeof(hardwareSensors) / sizeof(hardwareSensors[0]); ++i) {
    HardwareSensor* hardwareSensor = (HardwareSensor*)hardwareSensors[i];
    if (hardwareSensor->IsHardwareAccelerationSensor()) {
      HardwareAccelerationSensor* hardwareAccelerationSensor = (HardwareAccelerationSensor*)hardwareSensors[i];
      Serial.print(" - ");
      Serial.print(hardwareSensor->GetName());
      Serial.print(": x:");
      Serial.print(hardwareAccelerationSensor->AccelerationX);
      Serial.print(" m/s2, y: ");
      Serial.print(hardwareAccelerationSensor->AccelerationY);
      Serial.print(" m/s2, z: ");
      Serial.print(hardwareAccelerationSensor->AccelerationZ);
      Serial.println(" m/s2");
    }
  }

  Serial.println("\nGyro values:");
  for (int i = 0; i < sizeof(hardwareSensors) / sizeof(hardwareSensors[0]); ++i) {
    HardwareSensor* hardwareSensor = (HardwareSensor*)hardwareSensors[i];
    if (hardwareSensor->IsHardwareGyroSensor()) {
      HardwareGyroSensor* hardwareGyroSensor = (HardwareGyroSensor*)hardwareSensors[i];
      Serial.print(" - ");
      Serial.print(hardwareSensor->GetName());
      Serial.print(": x:");
      Serial.print(hardwareGyroSensor->GyroX);
      Serial.print(" deg/s, y: ");
      Serial.print(hardwareGyroSensor->GyroY);
      Serial.print(" deg/s, z: ");
      Serial.print(hardwareGyroSensor->GyroZ);
      Serial.println(" deg/s");
    }
  }

  Serial.println("\nMagnetic values:");
  for (int i = 0; i < sizeof(hardwareSensors) / sizeof(hardwareSensors[0]); ++i) {
    HardwareSensor* hardwareSensor = (HardwareSensor*)hardwareSensors[i];
    if (hardwareSensor->IsHardwareMagneticSensor()) {
      HardwareMagneticSensor* hardwareMagneticSensor = (HardwareMagneticSensor*)hardwareSensors[i];
      Serial.print(" - ");
      Serial.print(hardwareSensor->GetName());
      Serial.print(": x:");
      Serial.print(hardwareMagneticSensor->MagneticX);
      Serial.print(" uT, y: ");
      Serial.print(hardwareMagneticSensor->MagneticY);
      Serial.print(" uT, z: ");
      Serial.print(hardwareMagneticSensor->MagneticZ);
      Serial.println(" uT");
    }
  }

  Serial.println("\nAir pressure values:");
  for (int i = 0; i < sizeof(hardwareSensors) / sizeof(hardwareSensors[0]); ++i) {
    HardwareSensor* hardwareSensor = (HardwareSensor*)hardwareSensors[i];
    if (hardwareSensor->IsHardwareAirPressureSensor()) {
      HardwareAirPressureSensor* hardwareAirPressureSensor = (HardwareAirPressureSensor*)hardwareSensors[i];
      Serial.print(" - ");
      Serial.print(hardwareSensor->GetName());
      Serial.print(": ");
      Serial.print(hardwareAirPressureSensor->AirPressure);
      Serial.print(" hPa, ");
      Serial.print(hardwareAirPressureSensor->RelativeAltitude);
      Serial.println(" m");
    }
  }

  //void* ptr = &t40t;

  //HardwareTemperatureSensor x = *(HardwareTemperatureSensor*)ptr;
  //Serial.println(x.Temperature);

  //Serial.println(&foo->Temperature);
  //Serial.println( (*(HardwareTemperatureSensor*)ptr).       );

  HardwareAccelerationSensor *accelerationSensor = &lsm6ds3;
  HardwareGyroSensor *gyroSensor = &lsm6ds3;
  HardwareMagneticSensor *magneticSensor = &lsm303m;
  orientationSensor.Setup(accelerationSensor, gyroSensor, magneticSensor);

  Serial.println("\nSetup completed! Start operating...\n");
  /*
    Serial.println(bmp180.Temperature);

    HardwareTemperatureSensor* temp1 =&bmp180;
    HardwareTemperatureSensor temp2 =bmp180;
    Serial.println(temp2.Temperature);

    HardwareTemperatureSensor* temp = (HardwareTemperatureSensor*)hardwareSensors[0];
    Serial.println(temp->Temperature);*/

  HardwareTemperatureSensor* temperatureSensors[2] =
  {
    &bmp180, &bmp280
  };

  HardwareAirPressureSensor* airpressureSensors[2] =
  {
    &bmp180, &bmp280
  };
  
  plotter.SetTemperatureSensors(temperatureSensors);
  plotter.SetAirPressureSensors(airpressureSensors);
  plotter.Loop();
  plotter.Loop();
  plotter.Loop();

  
 /* HardwareTemperatureSensor* temperatureSensors2;
  temperatureSensors2 = temperatureSensors;

  HardwareAirPressureSensor *pressSensors[] =
  {
    &bmp180, &bmp280
  };
 Serial.println(pressSensors[0]->GetName());
 Serial.println(pressSensors[1]->GetName());
 Serial.println(pressSensors[0]->AirPressure);
 Serial.println(pressSensors[1]->AirPressure);*/


  
  /*HardwareAirPressureSensor *pressSensors2;
  pressSensors2 = pressSensors;



  Serial.println(bmp180.GetName());
  Serial.println(bmp280.GetName());
  Serial.println(temperatureSensors[0].GetName());
  Serial.println(temperatureSensors[1].GetName());
  HardwareAirPressureSensor *foo;
  foo = &bmp180;
  Serial.println(foo->GetName());
  
  
  Serial.println(temperatureSensors[0].Temperature);
  Serial.println(temperatureSensors[1].Temperature);

  Serial.println(pressSensors[0].AirPressure);
  Serial.println(pressSensors[1].AirPressure);

  Serial.println(temperatureSensors2[0].GetName());
  Serial.println(temperatureSensors2[1].GetName());
  
  Serial.println(temperatureSensors2[0].Temperature);
  Serial.println(temperatureSensors2[1].Temperature);

  Serial.println(pressSensors2[0].AirPressure);
  Serial.println(pressSensors2[1].AirPressure);*/


  Serial.println("*************************************************");


}

void loop()
{
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
  //telemetry.send();

  plotter.Loop();

  /*bmp180.UpdateSensorData();
    bmp280.UpdateSensorData();
    lsm303m.UpdateSensorData();
    lsm303a.UpdateSensorData();
    mpu6050.UpdateSensorData();
    hmc5883l.UpdateSensorData();
    lsm6ds3.UpdateSensorData();
    t40t.UpdateSensorData();*/

  /*for (int i = 0; i < sizeof(hardwareSensors) / sizeof(hardwareSensors[0]); ++i) {
    HardwareSensor* hardwareSensor = (HardwareSensor*)hardwareSensors[i];
    hardwareSensor->UpdateSensorData();
    }*/
  //Serial.print("bpm180:"); Serial.print(bmp180.RelativeAltitude); Serial.print(" bpm280:"); Serial.println(bmp280.RelativeAltitude);
  //Serial.print("mpu6050:");Serial.print(mpu6050.Temperature);Serial.print(" bpm180:");Serial.print(bmp180.Temperature);Serial.print(" bpm280:");Serial.println(bmp280.Temperature);
}
