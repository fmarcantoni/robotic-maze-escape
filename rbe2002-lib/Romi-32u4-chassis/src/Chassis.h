#pragma once

#include <Arduino.h>
#include <Romi32U4Motors.h>
#include <LSM6.h>

struct Pose
{
    float x = 0;
    float y = 0;
    float theta = PI/2;
};

//#define constrain(theta, -PI, PI);

class Chassis
{
protected:

    float wheel_track = 14.1488;//14.9; //cm 14.1488333
    float wheel_diam = 6.9875; //7.0; //cm 6.9875
    float ticks_per_rotation = 1440.0; // from the datasheet
    float cmPerEncoderTick = 3.1416 * wheel_diam / ticks_per_rotation;
    float robotRadius = wheel_track / 2.0;

public:
    uint8_t readyToPID = 0;

    Pose currentPose;
    Pose previousPose;

    Pose destinationPose;

    float heading = 0;

    LSM6 imu;
    float estimatedPitchAngle;
    float estimatedTurnAngle;
    float gyroBias = 0;

    Chassis(void);

    void init(void);
    bool loop(void);
    void update(void);
    bool checkForNewIMUData(void);
    float updatePitch(void);
    float turnAngle(void);
    void updatePose(void);


    void setMotorEfforts(int16_t left, int16_t right) {leftMotor.setMotorEffort(left); rightMotor.setMotorEffort(right);}

    void setMotorTargetSpeeds(float leftTicksPerInterval, float rightTicksPerInterval);
    void setWheelTargetSpeeds(float leftSpeed, float rightSpeed);
};


extern Chassis chassis;