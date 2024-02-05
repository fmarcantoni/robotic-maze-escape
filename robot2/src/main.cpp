// Main file for the IR BUTTON FINDER ROBOT, in which, apart from the set up, we are running our state machine in the main loop in the function "navigating"

#include <Arduino.h>
#include <robot.h>
#include <ir_codes.h>
#include <IRdecoder.h>
#include <HC-SR04.h>
#include <IRDirectionFinder.h>

// The pin for the IR remote for the IR Positioning Camera Robot is 14
#define IR_PIN 14
IRDecoder decoder(IR_PIN);


// set up
void setup() 
{
    Serial.begin(115200);
    delay(500);

    Serial.println("setup()");
    
    Serial1.begin(115200);

    decoder.init();

    initialize();

    Serial.println("/setup()");
}

void loop() 
{
    /**
     * Chassis::loop() returns true when the motor control loop fires. We can use that timer to trigger
     * any number of processes that we want to run on the same schedule, for example, the line following
     * controller.
    */
    chassis.loop();
    
    /**
     * But we can also process asynchronous events, such as IR remote presses or distance sensor readings.
    */

    int16_t keyCode = decoder.getKeyCode();
    if(keyCode != -1) handleKeyCode(keyCode);

    //running the state machine
    navigating();

    }