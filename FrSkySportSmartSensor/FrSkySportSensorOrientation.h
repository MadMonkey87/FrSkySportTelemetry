#ifndef _FRSKY_SPORT_SENSOR_ORIENTATION_H_
#define _FRSKY_SPORT_SENSOR_ORIENTATION_H_

#include "FrSkySportSensor.h"
#include <Kalman.h>
#include <MadgwickAHRS.h>

#define ORIENTATION_DEFAULT_ID ID23

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
  void setup();
  void loop();

protected:
    virtual void readSensorData();
    virtual uint16_t getSampleRate();
    bool sensorInitialized;           // true if setting up the sensor was successful
    double accX, accY, accZ;          // acceleration data of the sensor
    double gyroX, gyroY, gyroZ;       // gyro data of the sensor

private:
  void readAndCalculate();
  double getRoll();
  double getPitch();
  double getGForces();
    uint32_t processingTime;    // next time when the sensor data gets read and calculated
    double pitchOffset, rollOffset;
    uint32_t deltaTime; //used to determine the exact dt passed between sensor events
    double gyroXrate; //deg/s
    double gyroYrate; //deg/s
    double gyroXangle, gyroYangle; // Angle calculate using the gyro only
    double compAngleX, compAngleY; // Calculated angle using a complementary filter
    double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
    Kalman kalmanX;
    Kalman kalmanY;
    Madgwick filter;

};

#endif
