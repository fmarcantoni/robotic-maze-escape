// Final Code for IR Emitter Romi
// RBE 2002, D23, Team 5 
/**
 * @file robot.cpp
 * @include main.cpp, Chassis.h, HC-SR04.h, ir-codes.h
*/

//Header files
#include <robot.h>
#include <Chassis.h>
#include <HC-SR04.h>
#include <ir_codes.h>

// define constant(s)
#define LED_PIN 11
float targetDist = 10;
int xCoord;
int yCoord;
int currentDirection;
int AprilTag_Orient;
int AprilTag_X;
int AprilTag_Y;
uint8_t SecretCode_X;
uint8_t SecretCode_Y;
uint8_t SecretCode_Orientation;
bool SecretCodeEmitted;

// defines ultrasonic rangefinder pins
HC_SR04 hc_sr04(4, 13);

void ISR_HC_SR04(void) {hc_sr04.ISR_echo();}

ROBOT_STATE robotState = ROBOT_IDLE;

// creates objects
Chassis chassis;


// initialize function
void initialize(void){
  chassis.init();
  hc_sr04.init(ISR_HC_SR04);
  pinMode(13, OUTPUT);
}

// stopping function (E-stop)
void idle(void){
  Serial.println("Idling!");
  chassis.setMotorEfforts(0, 0);
  robotState = ROBOT_IDLE;
}

// keyCode functions, by button pressed
String keyCodeString;
void handleKeyCode(int16_t keyCode){
  if (keyCode == ENTER_SAVE)
    idle();

  switch (keyCode){
  case UP_ARROW:
    Serial.println("up");
    chassis.setWheelTargetSpeeds(5, 5);
    robotState = ROBOT_DRIVING;
    break;
  case RIGHT_ARROW:
    chassis.setMotorTargetSpeeds(10, -10);
    robotState = ROBOT_RIGHT;
    break;
  case DOWN_ARROW:
    chassis.setMotorTargetSpeeds(-10, -10);
    robotState = ROBOT_DRIVING;
    break;
  case LEFT_ARROW:
    chassis.setMotorTargetSpeeds(-10, 10);
    robotState = ROBOT_LEFT;
    break;
  case REWIND:
    robotState = ROBOT_STANDOFF;
    break;
  
  // Button pressed to begin IR emitter state machine in full demo
  case NUM_1:
    robotState = ROBOT_LOADING;
    xCoord = 0;
    yCoord = 0;
    currentDirection = 0;
    SecretCodeEmitted = false;
    AprilTag_Orient = -1;
    AprilTag_X = -1;
    AprilTag_Y = -1;
    Serial.println("Start");
    navigating();
    break;
  case NUM_2:
    
    delay(2000);
    SecretCode_X = '2';
    SecretCode_Y = '4';
    SecretCode_Orientation = '3';
    uint32_t code = codeCreate();
    emitIRCode(code);
  break;
}
}

//-------------------------------------------------------------------------------
// LOCATION UPDATE FUNCTIONS

void updateDirectionRightTurn(){
  // 1 = North, 2 = East, 3 = South and 4 = West
  if (currentDirection == 3){currentDirection = 0;}
  else {currentDirection = currentDirection + 1;}  

  Serial.print("DIRECTION   ");
  Serial.println(currentDirection);
}

void updateDirectionLeftTurn(){
  // 1 = North, 2 = East, 3 = South and 4 = West
  if (currentDirection == 0) {currentDirection = 3;}
  else {currentDirection = currentDirection - 1;}  

  Serial.print("DIRECTION   ");
  Serial.println(currentDirection);
}

// Records updated location and sends to MQTT
void logLocation(){
  if (currentDirection == 0) {yCoord++;}
  else if (currentDirection == 1) {xCoord++;}
  else if (currentDirection == 2) {yCoord = yCoord - 1;}
  else  if (currentDirection == 3) {xCoord = xCoord - 1;}
  String openParen = ("(");
  String comma = (",");
  String closeParen = (")");
  String printString = (openParen + xCoord + comma + yCoord + closeParen);

  Serial1.println("(x,y)" + String(':') + String(printString));
  Serial.println(printString);
}

//-------------------------------------------------------------------------
// NAVIGATION FUNCTIONS

void line_following(){
  if(robotState == ROBOT_NAVIGATION){
  // local variables
  float error, effort;
  float speed = 40.0;
  float Kp = 0.2;
  float left = analogRead(A3);  // left sensor reading
  float right = analogRead(A4); // right sensor reading
  error = right - left; // calculate the error
  effort = Kp * error;
  chassis.setMotorEfforts(speed - effort, speed + effort);
  }
}

// checks if robot has reached an intersection in line following
bool checkForIntersection(void) {
  // local variables
  bool intersection;
  // thresholds found based on testing
  float leftThreshold = 300.0;
  float rightThreshold = 550.0;

  float left = analogRead(A3);  // left sensor reading
  float right = analogRead(A4); // right sensor reading

  if(robotState == ROBOT_NAVIGATION){
    if (left <= leftThreshold && right <= rightThreshold) {intersection = true;}
    else {intersection = false;}
  }
  return intersection;
}

// function used to move past intersection,
// if checkForIntersection is true in state machine
void handleIntersection(void) {
  chassis.setMotorEfforts(30, 30);
  setTimer(1700);
  robotState = ROBOT_CENTERING;    
}

//-----------------------------------------------------------------------------------------------
// HANDLE RECIEVING FROM MQTT

// define variable
String serString1;

bool checkSerial1(){
  bool retVal = false;
    while(Serial1.available()){
        char c = Serial1.read();
        serString1 += c;
        if(c == '\n') retVal = true;
    }
    return retVal;
}

// function to receive april tag ID and orientation from MQTT
void recieveAprilTag(){
  String searchingTopic = ("AprilTag_Orient");
  String searchingTopic2 = ("AprilTag_ID");
  String recieveTopic = serString1.substring(0, serString1.indexOf(':'));

  int startMessage = serString1.indexOf(':') + 1;
  String recieveMessage = serString1.substring(startMessage);

  if(searchingTopic.equals(recieveTopic)){
    AprilTag_Orient = recieveMessage.toInt();
  }
  if (searchingTopic2.equals(recieveTopic)){
    AprilTag_Orient = recieveMessage.toInt();

    int startMessageX = serString1.indexOf(':') + 1;
    int startMessageY = serString1.indexOf(':') + 2;
    String recieveMessageX = serString1.substring(startMessageX, startMessageY);
    String recieveMessageY = serString1.substring(startMessageY);
    AprilTag_X = recieveMessageX.toInt();
    AprilTag_Y = recieveMessageY.toInt();
  }

  serString1 = "";
}

// function to receive escape door code from MQTT
void recieveSecretCode(){
  String searchingTopic = ("SecretCode_X");
  String searchingTopic2 = ("SecretCode_Y");
  String searchingTopic3 = ("SecretCode_Orient");
  String recieveTopic = serString1.substring(0, serString1.indexOf(':'));
  int startMessage = serString1.indexOf(':') + 1;

  if(searchingTopic.equals(recieveTopic)){
    String recieveMessageX = serString1.substring(startMessage);
    SecretCode_X = recieveMessageX.toInt();
  }

  if(searchingTopic2.equals(recieveTopic)){
    String recieveMessageY = serString1.substring(startMessage);
    SecretCode_Y = recieveMessageY.toInt();
  }

  if(searchingTopic3.equals(recieveTopic)){
    String recieveMessageOrientation = serString1.substring(startMessage);
    SecretCode_Orientation = recieveMessageOrientation.toInt();
  }

  serString1 = "";
}

// defines variable
bool NW;

// function to set direction of first movement from origin, based on april tag orientation
void setVars(void)
{
  if(AprilTag_Orient == 0 || AprilTag_Orient == 3){
          NW = true;
          updateDirectionRightTurn();
          chassis.setMotorEfforts(30, -30);
          setTimer(2500);
          Serial.println("NW");
          robotState = ROBOT_TURNING;
        }
        else if(AprilTag_Orient == 1 || AprilTag_Orient == 2){
          Serial.println("SE");
          NW = false;
          robotState = ROBOT_NAVIGATION;
        }
}

//-------------------------------------------------------------------------------
// APRIL TAG ORIENTATION HANDLING

// checks if robot is at correct orientation
bool checkOrientation(){
  bool retValue = false;
  if(currentDirection == AprilTag_Orient) {retValue = true;}
  return retValue;
}

// handler to turn to correct orientation
void handleOrientation(){
  if(NW == true){
    updateDirectionLeftTurn();
    chassis.setMotorEfforts(-30, 30);
    setTimer(2500);
    robotState = ROBOT_TURNING;
  }
  else{
    updateDirectionRightTurn();
    chassis.setMotorEfforts(30, -30);
    setTimer(2500);
    robotState = ROBOT_TURNING;
  }
}

//-------------------------------------------------------------------------------
// MQTT RECEIVING AND CONVERTING

// define variable
uint8_t BIT3 = 0x08;


// function to shift and convert code received from MQTT
uint32_t codeCreate()
{
  uint32_t code1Ascii = (uint32_t)SecretCode_X;
  uint32_t code2Ascii = (uint32_t)SecretCode_Y;
  uint32_t code3Ascii = (uint32_t)SecretCode_Orientation;

  uint32_t codeXOR = code1Ascii ^ code2Ascii ^ code3Ascii;

  uint32_t transmit = (code1Ascii << 24) | (code2Ascii << 16) | (code3Ascii << 8) | codeXOR;

  return transmit;
}

// function to emit escape door code
void emitIRCode(uint32_t transmit)
{
  // Initial data burst
  TCCR1A |= BIT3;
  delayMicroseconds(9000);
  // 4.5ms Space
  TCCR1A &= ~BIT3;
  delayMicroseconds(4500);

  // Address and logical inverse
  for (int i = 0; i < 32; i++)
  {
    TCCR1A |= BIT3;
    delayMicroseconds(562);
    TCCR1A &= ~BIT3;
    if(transmit & 1){delayMicroseconds(1687);}
    else{delayMicroseconds(562);}
    transmit >>= 1;
  }
  // Ending burst
  TCCR1A |= BIT3;
  delayMicroseconds(562);
  TCCR1A &= ~BIT3;
}

// function to emit code once in position and then leave through opened door
void handleNewDistanceReading(float distanceReading)
{
  float threshold = 15;
    if(robotState == ROBOT_FLASHING){
      if (distanceReading < threshold){
        if(SecretCodeEmitted == false){
          delay(2000);
          uint32_t code = codeCreate();
          emitIRCode(code);
          SecretCodeEmitted = true;
        }
      }
      else{
        chassis.setMotorEfforts(30,30);
        Serial.println("Escaping");
        setTimer(2000);
        robotState = ROBOT_ESCAPING;
      }
  }
} 

//-------------------------------------------------------------------------------
// TIMER FUNCTIONS

// define variable
uint32_t stopTime;

// function to set timer (for turning, etc.)
void setTimer(uint32_t duration) { stopTime = millis() + duration;}

// function to check if timer is expired
bool checkTimerExpired()
{
  bool TimerExpired = false;
  uint32_t currTime = millis();
  if (currTime >= stopTime)
  {
    TimerExpired = true;
  }
  return TimerExpired;
}

// function to handle expired timer, based on current robot state
void handleTimerExpired(){
  if(robotState == ROBOT_CENTERING){
    logLocation();
    Serial.println("WAITING");
    robotState = ROBOT_WAITING;
  }
  else if(robotState == ROBOT_TURNING){
    Serial.println("WAITING");
    robotState = ROBOT_WAITING;
  }
  else if(robotState == ROBOT_FLASHING){
    Serial.println("ESCAPING");
    robotState = ROBOT_ESCAPING;
  }
  else if(robotState == ROBOT_ESCAPING){
    chassis.setMotorEfforts(0,0);
    robotState = ROBOT_IDLE;
  }
}
//-------------------------------------------------------------------------------
//IR Emitter

void sendZero()
{
  // 562.5µs pulse burst followed by a 562.5µs space, with a total transmit time of 1.125ms
  TCCR1A |= BIT3;
  delayMicroseconds(562.5);
  TCCR1A &= ~BIT3;
  delayMicroseconds(562.5);
}

void sendOne()
{
  // 562.5µs pulse burst followed by a 1.6875ms space, with a total transmit time of 2.25ms
  TCCR1A |= BIT3;
  delayMicroseconds(562.5);
  TCCR1A &= ~BIT3;
  delayMicroseconds(1687.5);
}

//---------------------------------------------------------------------------
// ESCAPE COORDINATE CHECKERS AND HANDLERS

// checks if robot has reached target x-coordinate
bool checkX(){
  bool retValue = false;
  if(xCoord == AprilTag_X){
  retValue = true;
  Serial.println("ESCAPE X");
  }
  return retValue;
}

// handles x-coordinate checker, turns if necessary
void handleX(){
  if (currentDirection != 0){
    updateDirectionLeftTurn();
    chassis.setMotorEfforts(-30, 30);
    setTimer(2500);
    Serial.println("TURNING");
    robotState = ROBOT_TURNING;
  }
  else {robotState = ROBOT_NAVIGATION;}
}

// checks if robot has reached target y-coordinate
bool checkY(){
  bool retValue = false;
  if(yCoord == AprilTag_Y) {
    retValue = true;
    Serial.println("ESCAPE Y");}
  return retValue;
}

// handles y-coordinate checker, turns if necessary
void handleY(){
  Serial.println("Y HANDLE");
 if(currentDirection != 1){
    updateDirectionRightTurn();
    chassis.setMotorEfforts(30, -30);
    setTimer(2500);
    Serial.println("ROBOT TURNING");
    robotState = ROBOT_TURNING;
  }
  else robotState = ROBOT_NAVIGATION;
}

// handler function for both x and y escape coordinates
void checkEscapeCoordinate(){
  if(NW == true){
    if(checkX()){
      if(checkY()) {
        Serial.println("FLASHING");
        robotState = ROBOT_FLASHING;}
      else {handleX();}
    }
    else {
      Serial.println("NAV");
      robotState = ROBOT_NAVIGATION;}
    }
  else{
    if(checkY()){
      if(checkX()) {
        Serial.println("FLASHING");
        robotState = ROBOT_FLASHING;}
      else {handleY();}
    }
    else {
      Serial.println("NAV");
      robotState = ROBOT_NAVIGATION;}
    }
  }
 

//-----------------------------------------------------------------------------------------------
// NAVIGATION STATE MACHINE

// state machine function, what is called in main.cpp
void navigating()
{
    switch (robotState)
    {
      case ROBOT_LOADING:{ // enter if intersection is found
      if(checkSerial1()){
        recieveAprilTag();
        recieveSecretCode();
        }
        if(AprilTag_Orient != -1 && AprilTag_X != -1 && AprilTag_Y != -1) {setVars();}
      break;}

    case ROBOT_CENTERING:{ // enter if intersection is found
      if(checkTimerExpired()){
        handleTimerExpired();
      } 
      break;}

    case ROBOT_TURNING:{
      if(checkTimerExpired()){
        handleTimerExpired();
      } 
      break;}
    
    case ROBOT_WAITING:{
      checkEscapeCoordinate();
      break;
    }
    case ROBOT_NAVIGATION:{
      line_following();
      if(checkForIntersection()){handleIntersection();}
      break;
    }

    case ROBOT_ESCAPING:{
      if(checkTimerExpired()){
        handleTimerExpired();
      } 
      break;
    }

   case ROBOT_FLASHING:
   {
     if(checkOrientation()){
        chassis.setMotorEfforts(0,0);
        float distanceReading=0;
        bool hasNewReading = hc_sr04.getDistance(distanceReading);
        if(hasNewReading){
          handleNewDistanceReading(distanceReading);
        }
      }
      else{
        handleOrientation();
      }
      
    break;
   }
    default : break;
  }
}