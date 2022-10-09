#include "Plotter.h"
#include <Wire.h>

#include <TimeLib.h>

Sd2Card card;
SdVolume volume;
SdFile root;
String filename;

// change this to match your SD shield or module;
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
// Teensy audio board: pin 10
// Teensy 3.5 & 3.6 & 4.1 on-board: BUILTIN_SDCARD
// Wiz820+SD board: pin 4
// Teensy 2.0: pin 0
// Teensy++ 2.0: pin 20
const int chipSelect = 10;

bool sdCardReady = false;

void Plotter::Setup() {
  Serial.println("Initialiting plotter persistence...");

  Serial.print(" - date: "); printDigits(day()); Serial.print("."); printDigits(month()); Serial.print("."); Serial.println(year());
  Serial.print(" - time: "); printDigits(hour()); Serial.print(":"); printDigits(minute()); Serial.print(":"); printDigits(second()); Serial.println();

  if (!card.init(SPI_HALF_SPEED, chipSelect)) {
    Serial.println(" - no SD card was found (check wiring and chipSelect mode)");
    Serial.println("failed!");
    return;
  }

  if (!volume.init(card)) {
    Serial.println(" - could not find FAT16/FAT32 partition. Make sure you've formatted the card");
    Serial.println("failed!");
    return;
  }

  Serial.print(" - sd card type: ");
  switch (card.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.println("SD1");
      break;
    case SD_CARD_TYPE_SD2:
      Serial.println("SD2");
      break;
    case SD_CARD_TYPE_SDHC:
      Serial.println("SDHC");
      break;
    default:
      Serial.println("Unknown");
  }

  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  Serial.print(" - volume type is FAT");
  Serial.println(volume.fatType(), DEC);

  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  Serial.print(" - volume size (Kbytes): ");
  volumesize /= 2;
  Serial.println(volumesize);

  int i = 1;
  while (true) {
    filename = String("log_" + String(i, DEC) + ".csv");
    if (!SD.exists(filename.c_str())) {
      File logFile = SD.open(filename.c_str(), FILE_WRITE);
      Serial.print(" - log file: ");
      Serial.println(filename.c_str());
      logFile.close();

      break;
    }
    i++;
  }

  sdCardReady = true;
  Serial.println("successfull!");
}

bool headersWritten = false;

void Plotter::Log() {
  if (!sdCardReady) {
    return;
  }

  uint32_t now = millis();

  if (now > logTime) {
    logTime = now + 500;
    File logFile = SD.open(filename.c_str(), FILE_WRITE);

    if (!headersWritten) {

      headersWritten = true;

      logFile.print("timestamp;");

      for (unsigned int i = 0; i < temperatureSensorsCount; i++) {
        if (temperatureSensors[i]->IsReady()) {
          logFile.print(temperatureSensors[i]->GetName());
          logFile.print(" temperature (C°);");
        } else {
          Serial.print(temperatureSensors[i]->GetName());
          Serial.println("not ready");
        }
      }
      for (unsigned int i = 0; i < airPressureSensorsCount; i++) {
        if (airPressureSensors[i]->IsReady()) {
          logFile.print(airPressureSensors[i]->GetName());
          logFile.print(" air pressure (hPa);");
          logFile.print(airPressureSensors[i]->GetName());
          logFile.print(" relative altitude (m);");
        }
      }
      for (unsigned int i = 0; i < accelerationSensorsCount; i++) {
        if (accelerationSensors[i]->IsReady()) {
          logFile.print(accelerationSensors[i]->GetName());
          logFile.print("acceleration X;");
          logFile.print(accelerationSensors[i]->GetName());
          logFile.print("acceleration Y;");
          logFile.print(accelerationSensors[i]->GetName());
          logFile.print(" acceleration Z;");
          logFile.print(accelerationSensors[i]->GetName());
          logFile.print(" Gforce (g);");
        }
      }

      for (unsigned int i = 0; i < gyroSensorsCount; i++) {
        if (gyroSensors[i]->IsReady()) {
          logFile.print(gyroSensors[i]->GetName());
          logFile.print("gyro X;");
          logFile.print(gyroSensors[i]->GetName());
          logFile.print(" gyro Y;");
          logFile.print(gyroSensors[i]->GetName());
          logFile.print(" gyro Z;");
        }
      }

      for (unsigned int i = 0; i < magneticSensorsCount; i++) {
        if (magneticSensors[i]->IsReady()) {
          logFile.print(magneticSensors[i]->GetName());
          logFile.print(" magnetic X;");
          logFile.print(magneticSensors[i]->GetName());
          logFile.print(" magnetic Y;");
          logFile.print(magneticSensors[i]->GetName());
          logFile.print(" magnetic Z;");
        }
      }

    } else {

      logDigits(day(), logFile); logFile.print("."); logDigits(month(), logFile); logFile.print("."); logFile.print(year()); logFile.print(" ");
      logDigits(hour(), logFile); logFile.print(":"); logDigits(minute(), logFile); logFile.print(":"); logDigits(second(), logFile); logFile.print(";");

      for (unsigned int i = 0; i < temperatureSensorsCount; i++) {
        if (temperatureSensors[i]->IsReady()) {
          logFile.print(temperatureSensors[i]->Temperature);
          logFile.print(";");
        }
      }

      for (unsigned int i = 0; i < airPressureSensorsCount; i++) {
        if (airPressureSensors[i]->IsReady()) {
          logFile.print(airPressureSensors[i]->AirPressure);
          logFile.print(";");
          logFile.print(airPressureSensors[i]->RelativeAltitude);
          logFile.print(";");
        }
      }

      for (unsigned int i = 0; i < accelerationSensorsCount; i++) {
        if (accelerationSensors[i]->IsReady()) {
          logFile.print(accelerationSensors[i]->AccelerationX);
          logFile.print(";");
          logFile.print(accelerationSensors[i]->AccelerationY);
          logFile.print(";");
          logFile.print(accelerationSensors[i]->AccelerationZ);
          logFile.print(";");
          logFile.print(accelerationSensors[i]->GetGForces());
          logFile.print(";");
        }
      }

      for (unsigned int i = 0; i < gyroSensorsCount; i++) {
        if (gyroSensors[i]->IsReady()) {
          logFile.print(gyroSensors[i]->GyroX);
          logFile.print(";");
          logFile.print(gyroSensors[i]->GyroY);
          logFile.print(";");
          logFile.print(gyroSensors[i]->GyroZ);
          logFile.print(";");
        }
      }

      for (unsigned int i = 0; i < magneticSensorsCount; i++) {
        if (magneticSensors[i]->IsReady()) {
          logFile.print(magneticSensors[i]->MagneticX);
          logFile.print(";");
          logFile.print(magneticSensors[i]->MagneticY);
          logFile.print(";");
          logFile.print(magneticSensors[i]->MagneticZ);
          logFile.print(";");
        }
      }

    }

    logFile.println();
    logFile.close();
  }
}

void Plotter::SetTemperatureSensors(HardwareTemperatureSensor * temperatureSensors[], unsigned int count) {
  this->temperatureSensors = temperatureSensors;
  this->temperatureSensorsCount = count;
}

void Plotter::SetAirPressureSensors(HardwareAirPressureSensor * airPressureSensors[], unsigned int count) {
  this->airPressureSensors = airPressureSensors;
  this->airPressureSensorsCount = count;
}

void Plotter::SetAccelerationSensors(HardwareAccelerationSensor * accelerationSensors[], unsigned int count) {
  this->accelerationSensors = accelerationSensors;
  this->accelerationSensorsCount = count;
}

void Plotter::SetGyroSensors(HardwareGyroSensor * gyroSensors[], unsigned int count) {
  this->gyroSensors = gyroSensors;
  this->gyroSensorsCount = count;
}

void Plotter::SetMagneticSensors(HardwareMagneticSensor * magneticSensors[], unsigned int count) {
  this->magneticSensors = magneticSensors;
  this->magneticSensorsCount = count;
}

void Plotter::PrintDetails() {

  if (sdCardReady) {
    Serial.println("Persist: YES");
  } else {
    Serial.println("Persist: NO");
  }

  Serial.println("Temperature Sensors: ");
  for (unsigned int i = 0; i < temperatureSensorsCount; i++) {
    if (temperatureSensors[i]->IsReady()) {
      Serial.print(" - ");
      Serial.print(temperatureSensors[i]->GetName());
      Serial.print(": ");
      Serial.print(temperatureSensors[i]->Temperature);
      Serial.println(" C°");
    } else {
      Serial.print(" - ");
      Serial.print(temperatureSensors[i]->GetName());
      Serial.println(": not ready!");
    }
  }

  Serial.println();
  Serial.println("Air pressure Sensors: ");
  for (unsigned int i = 0; i < airPressureSensorsCount; i++) {
    if (airPressureSensors[i]->IsReady()) {
      Serial.print(" - ");
      Serial.print(airPressureSensors[i]->GetName());
      Serial.print(": ");
      Serial.print(airPressureSensors[i]->AirPressure);
      Serial.print(" hPa -> ");
      Serial.print(airPressureSensors[i]->RelativeAltitude);
      Serial.println(" m");
    } else {
      Serial.print(" - ");
      Serial.print(airPressureSensors[i]->GetName());
      Serial.println(": not ready!");
    }
  }

  Serial.println();
  Serial.println("Acceleration Sensors: ");
  for (unsigned int i = 0; i < accelerationSensorsCount; i++) {
    if (accelerationSensors[i]->IsReady()) {
      Serial.print(" - ");
      Serial.print(accelerationSensors[i]->GetName());
      Serial.print(": ");
      Serial.print(accelerationSensors[i]->AccelerationX);
      Serial.print(" m/s2 ");
      Serial.print(accelerationSensors[i]->AccelerationY);
      Serial.print(" m/s2 ");
      Serial.print(accelerationSensors[i]->AccelerationZ);
      Serial.println(" m/s2");
    } else {
      Serial.print(" - ");
      Serial.print(accelerationSensors[i]->GetName());
      Serial.println(": not ready!");
    }
  }

  Serial.println();
  Serial.println("Gyro Sensors: ");
  for (unsigned int i = 0; i < gyroSensorsCount; i++) {
    if (gyroSensors[i]->IsReady()) {
      Serial.print(" - ");
      Serial.print(gyroSensors[i]->GetName());
      Serial.print(": ");
      Serial.print(gyroSensors[i]->GyroX);
      Serial.print(" deg/s ");
      Serial.print(gyroSensors[i]->GyroY);
      Serial.print(" deg/s ");
      Serial.print(gyroSensors[i]->GyroZ);
      Serial.println(" deg/s");
    } else {
      Serial.print(" - ");
      Serial.print(gyroSensors[i]->GetName());
      Serial.println(": not ready!");
    }
  }

  Serial.println();
  Serial.println("Magnetic Sensors: ");
  for (unsigned int i = 0; i < magneticSensorsCount; i++) {
    if (magneticSensors[i]->IsReady()) {
      Serial.print(" - ");
      Serial.print(magneticSensors[i]->GetName());
      Serial.print(": ");
      Serial.print(magneticSensors[i]->MagneticX);
      Serial.print(" uT ");
      Serial.print(magneticSensors[i]->MagneticY);
      Serial.print(" uT ");
      Serial.print(magneticSensors[i]->MagneticZ);
      Serial.println(" uT");
    } else {
      Serial.print(" - ");
      Serial.print(magneticSensors[i]->GetName());
      Serial.println(": not ready!");
    }
  }
}

void Plotter::PlotTemperatures() {
  bool isFirstSensor = true;
  for (unsigned int i = 0; i < temperatureSensorsCount; i++) {
    if (temperatureSensors[i]->IsReady()) {
      if (isFirstSensor) {
        isFirstSensor = false;
      } else {
        Serial.print(",");
      }
      Serial.print(temperatureSensors[i]->GetName());
      Serial.print(": ");
      Serial.print(temperatureSensors[i]->Temperature);
    }
  }
  Serial.println();
}

void Plotter::PlotAirPressures() {
  bool isFirstSensor = true;
  for (unsigned int i = 0; i < airPressureSensorsCount; i++) {
    if (airPressureSensors[i]->IsReady()) {
      if (isFirstSensor) {
        isFirstSensor = false;
      } else {
        Serial.print(",");
      }
      Serial.print(airPressureSensors[i]->GetName());
      Serial.print(": ");
      Serial.print(airPressureSensors[i]->AirPressure);
    }
  }
  Serial.println();
}

void Plotter::PlotRealativeAltitudes() {
  bool isFirstSensor = true;
  for (unsigned int i = 0; i < airPressureSensorsCount; i++) {
    if (airPressureSensors[i]->IsReady()) {
      if (isFirstSensor) {
        isFirstSensor = false;
      } else {
        Serial.print(",");
      }
      Serial.print(airPressureSensors[i]->GetName());
      Serial.print(": ");
      Serial.print(airPressureSensors[i]->RelativeAltitude);
    }
  }
  Serial.println();
}

void Plotter::PlotAccelerationValues() {
  bool isFirstSensor = true;
  for (unsigned int i = 0; i < accelerationSensorsCount; i++) {
    if (accelerationSensors[i]->IsReady()) {
      if (isFirstSensor) {
        isFirstSensor = false;
      } else {
        Serial.print(",");
      }
      Serial.print(accelerationSensors[i]->GetName());
      Serial.print("_ACC_X: ");
      Serial.print(accelerationSensors[i]->AccelerationX);
      Serial.print(",");
      Serial.print(accelerationSensors[i]->GetName());
      Serial.print("_ACC_Y: ");
      Serial.print(accelerationSensors[i]->AccelerationY);
      Serial.print(",");
      Serial.print(accelerationSensors[i]->GetName());
      Serial.print("_ACC_Z: ");
      Serial.print(accelerationSensors[i]->AccelerationZ);
    }
  }
  Serial.println();
}

void Plotter::PlotGForceValues() {
  bool isFirstSensor = true;
  for (unsigned int i = 0; i < accelerationSensorsCount; i++) {
    if (accelerationSensors[i]->IsReady()) {
      if (isFirstSensor) {
        isFirstSensor = false;
      } else {
        Serial.print(",");
      }
      Serial.print(accelerationSensors[i]->GetName());
      Serial.print("_GFORCE: ");
      Serial.print(accelerationSensors[i]->GetGForces());
    }
  }
  Serial.println();
}

void Plotter::PlotGyroValues() {
  bool isFirstSensor = true;
  for (unsigned int i = 0; i < gyroSensorsCount; i++) {
    if (gyroSensors[i]->IsReady()) {
      if (isFirstSensor) {
        isFirstSensor = false;
      } else {
        Serial.print(",");
      }
      Serial.print(gyroSensors[i]->GetName());
      Serial.print("_GYRO__X: ");
      Serial.print(gyroSensors[i]->GyroX);
      Serial.print(",");
      Serial.print(gyroSensors[i]->GetName());
      Serial.print("_GYRO__Y: ");
      Serial.print(gyroSensors[i]->GyroY);
      Serial.print(",");
      Serial.print(gyroSensors[i]->GetName());
      Serial.print("_GYRO__Z: ");
      Serial.print(gyroSensors[i]->GyroZ);
    }
  }
  Serial.println();
}

void Plotter::PlotMagneticValues() {
  bool isFirstSensor = true;
  for (unsigned int i = 0; i < magneticSensorsCount; i++) {
    if (magneticSensors[i]->IsReady()) {
      if (isFirstSensor) {
        isFirstSensor = false;
      } else {
        Serial.print(",");
      }
      Serial.print(magneticSensors[i]->GetName());
      Serial.print("_MAG__X: ");
      Serial.print(magneticSensors[i]->MagneticX);
      Serial.print(",");
      Serial.print(magneticSensors[i]->GetName());
      Serial.print("_MAG__Y: ");
      Serial.print(magneticSensors[i]->MagneticY);
      Serial.print(",");
      Serial.print(magneticSensors[i]->GetName());
      Serial.print("_MAG__Z: ");
      Serial.print(magneticSensors[i]->MagneticZ);
    }
  }
  Serial.println();
}

void Plotter::printDigits(int digits) {
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void Plotter::logDigits(int digits, File logFile) {
  if (digits < 10)
    logFile.print('0');
  logFile.print(digits);
}
