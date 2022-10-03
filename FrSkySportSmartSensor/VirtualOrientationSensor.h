#ifndef _VIRTUAL_ORIENTATION_SENSOR_H_
#define _VIRTUAL_ORIENTATION_SENSOR_H_

#include "HardwareAccelerationSensor.h"
#include "HardwareGyroSensor.h"
#include "HardwareMagneticSensor.h"

#include <Kalman.h>
#include <MadgwickAHRS.h>

#include "Arduino.h"

class VirtualOrientationSensor
{
  public:
    void Setup(HardwareAccelerationSensor* accelerationSensor, HardwareGyroSensor* gyroSensor =  NULL, HardwareMagneticSensor* magneticSensor =  NULL);
    void ReadAndCalculate();

  protected:
    HardwareAccelerationSensor* accelerationSensor;
    HardwareGyroSensor* gyroSensor;
    HardwareMagneticSensor* magneticSensor;

  private:
    double getRollFromAcceleration();
    double getPitchFromAcceleration();
    
    uint32_t processingTime;    // next time when the sensor data gets read and calculated
    uint32_t deltaTime; //used to determine the exact dt passed between sensor events
        
    double pitchOffset, rollOffset;
    
    double gyroRollRate, gyroPitchRate; //deg/s
    
    double accelerationRollAngle, accelerationPitchAngle; //Calculates angles using the raw acceleration values
    double gyroPitchAngle, gyroRollAngle; //Calculates angles using the raw gyro values
    double complementaryRollAngle, complementaryPitchAngle; // Calculated angles using a complementary filter
    double kalmanRollAngle, kalmanPitchAngle; // Calculated angles using a Kalman filter
    
    Kalman kalmanRoll, kalmanPitch;
    Madgwick filter;

};

#endif
