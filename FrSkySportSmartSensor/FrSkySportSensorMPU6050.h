#ifndef _FRSKY_SPORT_SENSOR_MPU6050_H_
#define _FRSKY_SPORT_SENSOR_MPU6050_H_

#include "FrSkySportSensor.h"
#include <Kalman.h>
#include <Adafruit_MPU6050.h>

#define MPU6050_DEFAULT_ID ID22
#define MPU6050_DATA_COUNT 8
#define MPU6050_ACC_X_DATA_ID 0x0700
#define MPU6050_ACC_Y_DATA_ID 0x0710
#define MPU6050_ACC_Z_DATA_ID 0x0720
#define MPU6050_GFORCE_DATA_ID 0x0740
#define MPU6050_PITCH_DATA_ID 0x0760
#define MPU6050_ROLL_DATA_ID 0x0780
#define MPU6050_PITCHSPEED_DATA_ID 0x0781
#define MPU6050_ROLLSPEED_DATA_ID 0x0782

#define MPU6050_DATA_PERIOD 50 //time interval in MS between sensor readings
#define MPU6050_PUSH_PERIOD 10 //time interval in MS between individual data pushes

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

class FrSkySportSensorMPU6050 : public FrSkySportSensor
{
  public:
    FrSkySportSensorMPU6050(SensorId id = MPU6050_DEFAULT_ID);
    void setup();
    void calibrate();
    void readAndCalculate();
    virtual uint16_t send(FrSkySportSingleWireSerial &serial, uint8_t id, uint32_t now);
    virtual uint16_t decodeData(uint8_t id, uint16_t appId, uint32_t data);
  private:
    double getRoll(sensors_event_t event);
    double getPitch(sensors_event_t event);
    double getGForces(sensors_event_t event);

    uint32_t processingTime;    // next time when zhe sensor data gets read and calculated
    bool sensorInitialized;       // true if setting up the sensor was successful

    double pitchOffset, rollOffset;
    double gyroXangle, gyroYangle; // Angle calculate using the gyro only
    double compAngleX, compAngleY; // Calculated angle using a complementary filter
    double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
    Kalman kalmanX;
    Kalman kalmanY;
    uint32_t deltaTime; //used to determine the exact dt passed between sensor events
    double gyroXrate; //deg/s
    double gyroYrate; //deg/s
    sensors_event_t acceleration;
    sensors_event_t gyro;
};

#endif
