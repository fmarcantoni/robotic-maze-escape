#pragma once

#include <Arduino.h>
#include <Chassis.h>

enum ROBOT_STATE
{   
    ROBOT_IDLE, 
    ROBOT_LEFT,
    ROBOT_RIGHT,
    ROBOT_DRIVING,
    ROBOT_LINING,

    ROBOT_CLIMBING,
    ROBOT_STANDOFF, 

    ROBOT_NAVIGATION,
    ROBOT_CENTERING,    //for centering on an intersection
    ROBOT_WAITING,    //waiting for an observation
    ROBOT_TURNING, 
    ROBOT_ESCAPING,
    ROBOT_FLASHING,
    ROBOT_INCLINE, 
    ROBOT_LOADING,
    ANALYZE_DATA,
    EMIT_CODE, 
    DRIVE_TO_COORD,
};

enum LED_STATE
{
    LED_ON,
    LED_OFF,
};


void initialize(void);

void idle(void);

void handleKeyCode(int16_t keyCode);

void processDistanceSensors(void);
//void handleNewDistanceReading(float distanceReading);

bool checkForObject();

bool checkForIntersection(void);
void handleIntersection();

bool checkCreepingTimerExpired();

void handleCreepingTimerExpired();

void handleTurn();

bool checkTurningTimerExpired();
void handleTurningTimerExpired();

//float checkForNewDistanceReading();

//bool checkTimerExpired(void);
//void handleTimerTimerExpired(void);

void handleNewDistanceReading(float distanceReading);

void updateDirection();
void logLocation();

void line_following(void);

void navigating();//ROBOT_STATE robotState);


void setTimer(uint32_t);
bool checkTimerExpired();
void handleTimerExpired();

bool checkTurnComplete();
void handleCompleteTurn();

float checkNewDistanceReadingAvailable();

void updateDirectionLeftTurn();
void updateDirectionRightTurn();

void handleY();

void recieveSecretCode();

void setLED();
void onLED();
void offLED();
bool checkForRamp();
void handleRamp();
bool checkForFlatSurface();
void handleFlatSurface();

void LEDTimer1();
void LEDTimer();

void codeOn();
void codeOff();
void codeEmit();

void sortData();
void analyzeData();
void sendZero();
void sendOne();
uint32_t codeCreate();
void emitIRCode(uint32_t transmit);
void emitDoorCode();
bool checkDoorOpen();
void handleDoorOpen();

bool checkSerial1();
void recieveAprilTag();
void recieveAprilTag_XY();

void setVars();

