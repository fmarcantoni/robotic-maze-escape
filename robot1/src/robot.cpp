// Coomplete code for RampBot functionality; state machine and all dependant functions

#include <robot.h>
#include <Chassis.h>
#include <ir_codes.h>
#include <openmv.h>

// Declaring global variables
int xCoord;
int yCoord;
int currentDirection;
int rampX;

ROBOT_STATE robotState = ROBOT_IDLE;


//Initialization
Chassis chassis;
void initialize(void) {
  chassis.init();
  pinMode(13, OUTPUT);
}

void idle(void){
  Serial.println("Idling!");
  chassis.setMotorEfforts(0, 0);
  robotState = ROBOT_IDLE;
}

//--------------------------------------------------------------------------------------------------------------------
// REMOTE CONTROLLER STATE MACHINE

String keyCodeString;
void handleKeyCode(int16_t keyCode){

  if (keyCode == ENTER_SAVE)
    idle();

  switch (keyCode) {
  case UP_ARROW:
    chassis.setWheelTargetSpeeds(8, 8);
    robotState = ROBOT_DRIVING;
    break;
  case RIGHT_ARROW:
    chassis.setWheelTargetSpeeds(8, -8);
    robotState = ROBOT_RIGHT;
    break;
  case DOWN_ARROW:
    chassis.setWheelTargetSpeeds(-8, -8);
    robotState = ROBOT_DRIVING;
    break;
  case LEFT_ARROW:
    chassis.setWheelTargetSpeeds(-8, 8);
    robotState = ROBOT_LEFT;
    break;
  case REWIND:
    robotState = ROBOT_STANDOFF;
    break;
  case NUM_3:
  // Set all updating values back to 0 when the function is restarted
    xCoord = 0;
    yCoord = 0;
    currentDirection = 0;
    rampX = 0;
    robotState = ROBOT_LOADING;
    navigating();
    break;
    }
}

//--------------------------------------------------------------------------------------------------------------------
// MQTT BROKER FUNCTIONS

// Send a properly formatted topic and message to MQTT broker
void sendMessage(const String& topic, const String& message) {
    Serial1.println(topic + String(':') + message);
}

// Retrieve a message from the MQTT broker and store it in the variable serString1.
String serString1;
bool checkSerial1(){
  bool retVal = false;
    while(Serial1.available())
    {
        char c = Serial1.read();
        serString1 += c;

        if(c == '\n') retVal = true;
    }
    return retVal;
}

// Identify if the message recieved from the broker is of the topic rampX; if it is, the message is stored as an interger in the variable rampX
void recieveRampX(){
  String searchingTopic = ("rampX");
  String recieveTopic = serString1.substring(0, serString1.indexOf(':'));
  
  int startMessage = serString1.indexOf(':') + 1;

  if(searchingTopic.equals(recieveTopic)){
    String recieveMessage = serString1.substring(startMessage);

    rampX = recieveMessage.toInt();
  }
  serString1 = "";
}

// Obtain the ID and Orientation of the AprilTag from the camera and send to MQTT broker;
// Orientation is sent as either 2 for south or 0 for north dependsnt on the angle recieved from the tag
void sendAprilTag(){
  OpenMV camera;
  uint8_t tagCount = camera.getTagCount();
  String doorOrient;
    if(tagCount) {
      AprilTagDatum tag;
      if(camera.readTag(tag)) {

        if (tag.rot <= 180){doorOrient = "2";}
        else {doorOrient = "0";}
        
        sendMessage("AprilTag_ID", String(tag.id));
        sendMessage("AprilTag_Orient", doorOrient);
      }
    }
}

//--------------------------------------------------------------------------------------------------------------------
// LOCATION LOGGING

// Update direction (stored in the currentDirection variable as 0 for North 1 for East 2 for South or 3 for West) when the robot takes a right turn
void updateDirectionForRightTurn() {
  if (currentDirection == 3) {currentDirection = 0;}
  else {currentDirection = currentDirection + 1;} 
}

// Update direction (stored in the currentDirection variable as 0 for North 1 for East 2 for South or 3 for West) when the robot takes a left turn
void updateDirectionForLeftTurn() {
  if (currentDirection == 0) {currentDirection = 3;}
  else {currentDirection = currentDirection - 1;} 
}

// Update the x and y coordinates (stored in the XCoord and YCoord variables respectively) and send the coordindinates to the MQTT broker
void logLocation() {
  if (currentDirection == 0) {yCoord++;}
  else if (currentDirection == 1) {xCoord++;}
  else if (currentDirection == 2) {yCoord = yCoord - 1;}
  else  if (currentDirection == 3) {xCoord = xCoord - 1;}

  String openParen = ("(");
  String comma = (",");
  String closeParen = (")");
  String printString = (openParen + xCoord + comma + yCoord + closeParen);

  sendMessage("rampBot/location", printString);
}

//--------------------------------------------------------------------------------------------------------------------
// BASE LEVEL NAVIGATION

// Line following functionality
void line_following() { 
  float error, effort;
  float speed = 6.0;
  float Kp = 0.015;

  float left = analogRead(A4); 
  float right = analogRead(A3);

  error = right - left;
  effort = Kp * error;

  chassis.setWheelTargetSpeeds(speed - effort, speed + effort);
}

// Identify intersections
bool checkForIntersection() { 
  bool intersection;
  float leftThreshold = 500.0;
  float rightThreshold = 500.0;

  float left = analogRead(A4);
  float right = analogRead(A3);

  if(robotState == ROBOT_NAVIGATION){
    if (left <= leftThreshold && right <= rightThreshold) {intersection = true;}
    else {intersection = false;}
  }
  return intersection;
}

// When an intersection is found, roll forward and set a timer and change the state to centering
void handleIntersection() {
  chassis.setWheelTargetSpeeds(8, 8);
  setTimer(1100);
  sendMessage("rampBot/State", "INTERSECTION");
  robotState = ROBOT_CENTERING;
}

//--------------------------------------------------------------------------------------------------------------------
// TIMER

uint32_t stopTime;

// Given the duration of the timer, identify the stop time by adding that duration to the value of millis at the time the function is called and store it in the variable stopTime
void setTimer(uint32_t duration) {stopTime = millis() + duration;}


// Compares current time to stopTime to identify if the timer has expired
bool checkTimerExpired(){
  bool TimerExpired = false;
  uint32_t currTime = millis();
  if (currTime >= stopTime) {TimerExpired = true;}

  return TimerExpired;
}

// Dependant on the state of the robot when the timer is checked,
void handleTimerExpired(){
  if(robotState == ROBOT_CENTERING){
    logLocation();
    robotState = ROBOT_WAITING;
  }
  else if(robotState == ROBOT_TURNING){
    robotState = ROBOT_NAVIGATION;
  }
  else if(robotState == ROBOT_NAVIGATION){
    sendMessage("rampBot/State", "ONRAMP");
    robotState = ROBOT_ONRAMP;
  }
  else if(robotState == ROBOT_PLATFORM){
    chassis.setWheelTargetSpeeds(0, 0);
    sendMessage("onPlatform", "1");
    robotState = ROBOT_APRILTAG;
  }
}

//--------------------------------------------------------------------------------------------------------------------
// RAMP IDENTIFICATION

int platformReadings = 0;
float rampThreshold = -15;
float platformThreshold = 2;
float prevAngle;

// Determine if the current x coordinate of the robot is the same as the rampX coordinate recieved from the broker
bool checkX(){
  bool retVal = false;
  if (xCoord == rampX) {retVal = true;}
  return retVal;
}

// If the x is correct but the robot is not facing north, turn right, set the timer, update direction, and set the state to turning; if the robot is facing north set state to navigating
void handleX(){
  if (currentDirection != 0){
    chassis.setWheelTargetSpeeds(-8, 8);
    setTimer(1600);
    updateDirectionForLeftTurn();
    sendMessage("rampBot/State", "TURNING");
    robotState = ROBOT_TURNING;
  }
  else {robotState = ROBOT_NAVIGATION;}
}

// Using pitch angle identification from chassis determine if the current and previous pitch angle reading is within the thresholds of the angle of the ramp
bool checkRamp(){
  bool ramp;
  float angle = chassis.updatePitch();

  if (angle < rampThreshold && prevAngle < rampThreshold){ramp = true;}
  else {ramp = false;}

  prevAngle = angle;
  return ramp;
}

//Set robot state to on ramp
void handleRamp() {robotState = ROBOT_ONRAMP;}

// Steady state error correction for platform identification, saves the number of consecutive readings within the platform threshold in the platformReadings variable
void platformSteadyState(float threshold){
  float angle = chassis.updatePitch();
  if (angle > threshold){platformReadings++;}
  else platformReadings = 0;
}

// Call steady state function to determine if there have been 8 consecutive readings within the platform thresholds
bool checkPlatform(){
  bool platform;
  platformSteadyState(platformThreshold);
  if(platformReadings == 8){platform = true;}
  else{platform = false;}

  return platform;
}

// When the platform is found, roll forward and set a timer and change the state to platform
void handlePlatform(){
  chassis.setWheelTargetSpeeds(6,6);
  setTimer(1250);
  sendMessage("rampBot/State", "PLATFORM");
  robotState = ROBOT_PLATFORM;
}

//--------------------------------------------------------------------------------------------------------------------
// RAMPBOT STATE MACHINE

void navigating() {
    switch (robotState) {

    // Initial state; once the rampX coordinate is recieved start movement by turning right
    case ROBOT_LOADING:{
      if(checkSerial1()){recieveRampX();}
      if(rampX){
        chassis.setWheelTargetSpeeds(8,-8);
        setTimer(1600);
        robotState = ROBOT_TURNING;
        updateDirectionForRightTurn();}
      break;}

    // Forward line following
    case ROBOT_NAVIGATION:{
      line_following();
      if(checkForIntersection()) {handleIntersection();}
      if(checkX() && checkRamp()){handleRamp();}
      break;}

    // When an intersection is found, center the robot over the intersect
    case ROBOT_CENTERING:{
      if(checkTimerExpired()) {handleTimerExpired();} 
      break;}

    // After centering, call checkX
    case ROBOT_WAITING:{
      if(checkX()){handleX();}
      else {robotState = ROBOT_NAVIGATION;}
      break;}

    // Call timer checker handler for turn
    case ROBOT_TURNING:{
      if(checkTimerExpired()) {handleTimerExpired();} 
      break;}

    // Line follow up ramp and look for platform
    case ROBOT_ONRAMP:{
      line_following();
      if(checkPlatform()){handlePlatform();}
      break;}

    // Call timer checker handler when robot is on platform to bring AprilTag in camera's view
    case ROBOT_PLATFORM:{
      if(checkTimerExpired()){handleTimerExpired();} 
      break;}

    // After arriving on the platform and rolling forward to make sure the AprilTag is in range of the camera, begin sending the tag information
    case ROBOT_APRILTAG:{
      sendAprilTag();
      break;
    }

    default : break;
  }
}
