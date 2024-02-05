// Call remote controller state machine as well as RampBot state machine

#include <Arduino.h>
#include <robot.h>
#include <ir_codes.h>
#include <IRdecoder.h>
#include <HC-SR04.h>


#define IR_PIN 17
IRDecoder decoder(IR_PIN);

void setup() 
{
    Serial.begin(115200);
    delay(500);
    Serial.println("setup()");
    decoder.init();
    Serial1.begin(115200);
    initialize();
    Serial.println("/setup()");
}

void loop() {
    chassis.loop();

    int16_t keyCode = decoder.getKeyCode();
    if(keyCode != -1) handleKeyCode(keyCode);

    navigating();
}