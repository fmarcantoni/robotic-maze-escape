#pragma once

#include <Arduino.h>
#include <Chassis.h>
#include <IRDirectionFinder.h>

enum ROBOT_STATE
{   
    ROBOT_IDLE, 
    ROBOT_LEFT,
    ROBOT_RIGHT,
    ROBOT_DRIVING,
    ROBOT_LINING,

    ROBOT_CLIMBING,
    ROBOT_STANDOFF, 

    ROBOT_NAVIGATION,   //robot navigating (line following)
    ROBOT_CENTERING,    //for centering on an intersection
    ROBOT_WAITING,      //waiting for an observation
    ROBOT_TURNING,      //whenever the robot makes a turn
    ROBOT_APRIL_TAG,   //waiting to recive from the broker the information that the second robot is in position to read the AprilTag
    ROBOT_PUSHING_BUTTON,   //tobot pushes the button
};


void initialize(void);

void idle(void);

void handleKeyCode(int16_t keyCode);

void updateDirection();

void logLocation();

void line_following(void);

bool checkForIntersection(void);

void handleIntersection();

void handleNewDistanceReading(float distanceReading);

void setTimer(uint32_t duration);

bool checkTimerExpired();

void handleTimerExpired();

bool checkIREmitter();

void handleIREmitter();

void sendMessage(const String& topic, const String& message);

bool checkSerial1();

bool receiveOnPlatform();

void handleSecondRobotOnPlatform();

void findRamp();

void navigating();