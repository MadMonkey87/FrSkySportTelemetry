#include "I2CScanner.h"
#include <Wire.h>

// Set I2C bus to use: Wire, Wire1, etc.
#define WIRE Wire

I2CScanner::I2CScanner()
{
}

void I2CScanner::scan()
{
    WIRE.begin();

    byte error, address;
    int nDevices;

    Serial.println("Scanning for I2C devices...");

    nDevices = 0;
    for (address = 1; address < 127; address++)
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        WIRE.beginTransmission(address);
        error = WIRE.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.print(address, HEX);
            Serial.println("  !");

            nDevices++;
        }
        else if (error == 4)
        {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
    {
        Serial.println("No I2C devices found\n");
    }
    else
    {
        Serial.println("scanning completed!\n");
    }

    WIRE.end();
}