#pragma once

#include <Arduino.h>
#include <Chassis.h>

// Declaration of states for state machine
enum ROBOT_STATE {   
    ROBOT_IDLE, 
    ROBOT_DRIVING,
    ROBOT_LEFT,
    ROBOT_RIGHT,
    ROBOT_STANDOFF,

    ROBOT_LOADING,
    ROBOT_NAVIGATION,
    ROBOT_CENTERING, 
    ROBOT_WAITING,
    ROBOT_TURNING, 
    ROBOT_ONRAMP,
    ROBOT_PLATFORM,
    ROBOT_APRILTAG,  
};

void initialize();

void idle();

void handleKeyCode(int16_t keyCode);

bool checkPlatform();
void handlePlatform();

bool checkForIntersection(void);
void handleIntersection();

void updateDirectionForRightTurn();
void updateDirectionForLeftTurn();
void logLocation();

void line_following(void);

void navigating();

void sendAprilTag();

void setTimer(uint32_t);
bool checkTimerExpired();
void handleTimerExpired();

bool checkSerial1();
void recieveRampX();

