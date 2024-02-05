// Motor control functions and pitch angle determination

#include <Arduino.h>
#include <Chassis.h>
#include <Romi32U4Motors.h>
#include <LSM6.h>

LeftMotor leftMotor;
RightMotor rightMotor;

Chassis::Chassis(void) {}

void Chassis::init(void)
{  
    noInterrupts(); 

    TCCR4A = 0x00; 
    TCCR4B = 0x0A; 
    TCCR4C = 0x04; 
    TCCR4D = 0x00; 

    TC4H = 2; OCR4C = 249;
    TIMSK4 = 0x04; 
    interrupts();
    Romi32U4Motor::init();
    imu.init();
}

bool Chassis::loop(void)
{
    bool retVal = false;
    if(readyToPID)
    {
        if(readyToPID > 1) Serial.println("Missed update in Chassis::loop()");

        update();
        readyToPID = 0;
        retVal = true;
    }

    return retVal;
}

void Chassis::update(void)
{
    leftMotor.update();
    rightMotor.update();

#ifdef __MOTOR_DEBUG__
    Serial.print('\n');
#endif
}

void Chassis::setMotorTargetSpeeds(float leftTicksPerInterval, float rightTicksPerInterval){
    leftMotor.setTargetSpeed(leftTicksPerInterval);
    rightMotor.setTargetSpeed(rightTicksPerInterval);
}

void Chassis::setWheelTargetSpeeds(float leftSpeed, float rightSpeed)
{
    float leftTicksPerInterval, rightTicksPerInterval;

    leftTicksPerInterval = (leftSpeed/cmPerEncoderTick)*0.02;
    rightTicksPerInterval = (rightSpeed/cmPerEncoderTick)*0.02;

    leftMotor.setTargetSpeed(leftTicksPerInterval);
    rightMotor.setTargetSpeed(rightTicksPerInterval);
}


ISR(TIMER4_OVF_vect) {
  leftMotor.calcEncoderDelta();
  rightMotor.calcEncoderDelta();

  chassis.readyToPID++;
}

bool Chassis::checkForNewIMUData(void){
    if(imu.getStatus() & 0x01) return true;
    return false;
}

// Use IMU to find the filtered pitch angle of the robot
float Chassis::updatePitch(void){
    float k=0.75;
    float e = 0.01;
    float ODRinHz = 208;

    imu.readAcc();
    imu.readGyro();

    float previousBias = gyroBias;

    float ObservedPitchAngle = (atan2(-imu.a.x, imu.a.z))*(180.0/PI);
    
    float changeInPitchAngle = imu.calculateChangeInPitchAngle(ODRinHz, imu.g.y, previousBias);
    float predictedAngle = estimatedPitchAngle + changeInPitchAngle;
    float filteredAngle =(1-k)*predictedAngle + ObservedPitchAngle*k;

    gyroBias = previousBias - (e/((1/ODRinHz)*0.0175))*(ObservedPitchAngle-predictedAngle);

    estimatedPitchAngle = filteredAngle;

    return filteredAngle;
    
}