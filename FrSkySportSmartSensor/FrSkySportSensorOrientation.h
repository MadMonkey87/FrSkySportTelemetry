#ifndef _FRSKY_SPORT_SENSOR_ORIENTATION_H_
#define _FRSKY_SPORT_SENSOR_ORIENTATION_H_

#include "FrSkySportSensor.h"

#include "HardwareAccelerationSensor.h"
#include "HardwareGyroSensor.h"
#include "HardwareMagneticSensor.h"

#include <Kalman.h>
#include <MadgwickAHRS.h>

#define ORIENTATION_DEFAULT_ID ID22
#define ORIENTATION_DATA_COUNT 8
#define ORIENTATION_ACC_X_DATA_ID 0x0700
#define ORIENTATION_ACC_Y_DATA_ID 0x0710
#define ORIENTATION_ACC_Z_DATA_ID 0x0720
#define ORIENTATION_GFORCE_DATA_ID 0x0740
#define ORIENTATION_PITCH_DATA_ID 0x0760
#define ORIENTATION_ROLL_DATA_ID 0x0780
#define ORIENTATION_PITCHSPEED_DATA_ID 0x0781
#define ORIENTATION_ROLLSPEED_DATA_ID 0x0782

#define ORIENTATION_DATA_PERIOD 50 //time interval in MS between sensor readings
#define ORIENTATION_PUSH_PERIOD 10 //time interval in MS between individual data pushes

class FrSkySportSensorOrientation : public FrSkySportSensor
{
  public:
    FrSkySportSensorOrientation(SensorId id = ORIENTATION_DEFAULT_ID);
    uint16_t send(FrSkySportSingleWireSerial &serial, uint8_t id, uint32_t now);
    uint16_t decodeData(uint8_t id, uint16_t appId, uint32_t data);
    void loop();
    void Setup(HardwareAccelerationSensor* accelerationSensor, HardwareGyroSensor* gyroSensor =  NULL, HardwareMagneticSensor* magneticSensor =  NULL);
    void readAndCalculate();
    
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
