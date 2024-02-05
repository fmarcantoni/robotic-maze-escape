#include <Arduino.h>
#include <robot.h>

/**
 * Most of the accessories, we put in robot.h/.cpp. We put the IR remote here because it's 
 * responsible for overall command and control -- it's not something that the robot uses directly
 * for control
*/
#include <ir_codes.h>
#include <IRdecoder.h>
#include <HC-SR04.h>

#define IR_PIN 14
IRDecoder decoder(IR_PIN);

void setup() 
{
  // This will initialize the Serial (Romi) and Serial1 (MQTT) at a baud rate of 115200 for prints
  Serial.begin(115200);
  delay(500);
  Serial1.begin(115200);

  // Starting I2C Bus
  Wire.begin();
  Wire.setClock(100000ul);

  // Initalize objects
  chassis.init();
  decoder.init();
  idle();
  pinMode(11, OUTPUT);
  Serial.println("/setup()");
}

void loop() 
{
    chassis.loop();

    int16_t keyCode = decoder.getKeyCode();
    if(keyCode != -1) handleKeyCode(keyCode);

    // state machine function, found in robot.cpp
    navigating();
    }