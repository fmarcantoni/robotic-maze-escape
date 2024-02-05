#include <robot.h>
#include <Chassis.h>
#include <HC-SR04.h>
#include <ir_codes.h>
#include <IRDirectionFinder.h>

//defining variables that we'll be using in our state machine

//x and y coordinates of the cell in which the robot is at and it's current direction
int xCoord;
int yCoord;
int currentDirection;

//IRDetected is true when the IR button has been detected
bool IRDetected;

//flag to check if at every intersection we checked if the IR button is at the robot's right
bool checkIRrecovery = false;

//flag used to communicate with the second robot, to detect if it's in position to read the AprilTag
bool OnPlatform = false;

IRDirectionFinder irFinder;

HC_SR04 hc_sr04(11, 13);

void ISR_HC_SR04(void)
{
  hc_sr04.ISR_echo();
}

ROBOT_STATE robotState = ROBOT_IDLE;

Chassis chassis;

void initialize(void)
{
  chassis.init();

  hc_sr04.init(ISR_HC_SR04);

  pinMode(13, OUTPUT);

  irFinder.begin();
}

void idle(void)
{
  Serial.println("Idling!");
  chassis.setMotorEfforts(0, 0);
  robotState = ROBOT_IDLE;
}

String keyCodeString; // this may come in handy later
void handleKeyCode(int16_t keyCode)
{
  Serial.println(keyCode);

  if (keyCode == ENTER_SAVE)
    idle();

  switch (keyCode)
  {
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

  case NUM_2:
    //when number two is pressed on the IR remote we initialize the coordinates of the robot, starting at cell (0,0), we set the current direction to be North, and setting that the IR has not been detected yet
    //after initializing we run our state machine starting at the robot state of navigation
    robotState = ROBOT_NAVIGATION;
    xCoord = 0;
    yCoord = 0;
    currentDirection = 0;
    IRDetected = false;
    navigating();
    break;
    }
}


//-------------------------------------------------------------------------------
// BASIC NAVIGATION FUNCTIONS

// whenever we make a right turn, we call updateDirection which updates the direction as follows: North -> East -> South -> West, and sends it to the MQTT broker
void updateDirection()
{
  // 0 = North, 1 = East, 2 = South and 3 = West
  if (currentDirection == 3){
    currentDirection = 0;
  }
  else {
    currentDirection = currentDirection + 1;
  }

  String openParen = ("(");
  String closeParen = (")");
  String printString = (openParen + currentDirection + closeParen);

  //we send the current direction to the broker
  Serial1.println("direction" + String(':') + String(printString));
}

//whenever we detect an intersection we update the coordinates of our robot depending on which direction we are facing and we send them to the MQTT broker
void logLocation()
{
  if (currentDirection == 0){
    yCoord++;
  }
  else if (currentDirection == 1)
 {
    xCoord++;
 }
 else if (currentDirection == 2){
    yCoord = yCoord - 1;
  }
 else if (currentDirection == 3){
    xCoord = xCoord - 1;
  }

  String openParen = ("(");
  String comma = (",");
  String closeParen = (")");
  String printString = (openParen + xCoord + comma + yCoord + closeParen);

  //we send the x and y coordinates to the broker
  Serial1.println("(x,y)" + String(':') + String(printString));

  //code for sending the pose at every intersection used in Lab 6

  //String poseString = (openParen + chassis.currentPose.x + comma + chassis.currentPose.y + comma + chassis.currentPose.theta + closeParen);
  //Serial1.println("pose" + String(':') + String(poseString));
}

//function used when the robot is in the state of NAVIGATION, which follows the line using a Proportional Controller
void line_following()
{
  if(robotState == ROBOT_NAVIGATION){

  float error, effort;
  float speed = 8.0;
  float Kp = 0.02;

  //we read the values of the line sensor connecting the left pin to pin A3 and the right pin to A4
  float left = analogRead(A3);  //left sensor reading
  float right = analogRead(A4); //right sensor reading

  // we calculate the error and then, using the Kp, we calculate the effort
  error = right - left;
  effort = Kp * error;

  //we set the speed of the left wheel to speed - effort because we are calulating the error subtracting the left reading and, therefore, the speed of the right wheel will be speed + effort
  chassis.setWheelTargetSpeeds(speed - effort, speed + effort);
  }
}


//-------------------------------------------------------------------------------
// INTERSECTION

//this function is keep being called in the NAVIGATION STATE in which we are line following, which checks if the robot is an intersection using the readings of the line sensor
bool checkForIntersection(void)
{
  // the returned value, which is true if we are on an intersection
  bool intersection;

  //the thresholds of the two pins on the line sensor, when our reading are below these thresholds it means that we are on an intersection
  float leftThreshold = 260.0;
  float rightThreshold = 100.0;

  float left = analogRead(A3);  // left sensor reading
  float right = analogRead(A4); // right sensor reading

  //it runs only if we are in the NAVIGATION state
  if(robotState == ROBOT_NAVIGATION){

    //if an intersection has been detected we set the flag to true
    if (left <= leftThreshold && right <= rightThreshold){
      intersection = true;
    }
    else{
      intersection = false;
    }
  }
  return intersection;
}

// if an intersection has been detected, handleIntersection is called, which set a timer and drives forward for that amount of time to center on the intersection. 
//then it sends the coordinates to the broker and move the robot in the CENTERING state
void handleIntersection(void)
{  
  chassis.setWheelTargetSpeeds(8, 8);
  setTimer(1100);
  logLocation();
  robotState = ROBOT_CENTERING;
}


//-------------------------------------------------------------------------------
//DISTANCE READING

//handleNewDistanceReading is called whenever we are in the WAITING state in which we go once we finished centering or turning
void handleNewDistanceReading(float distanceReading)
{
  float threshold = 22; // distance threshold

  if(robotState == ROBOT_WAITING){

    //if an object is too close
    if(distanceReading < threshold){

      //if the IR button has been detected and we are close to it, we stop, we send a message to the broker telling where the ramp is, and using the coordinates and direction at which we are, we set the variables of the secret code to then send to the third robot with the IR emitter; 
      //and we switch to the APRIL_TAG state in which we will be waiting for the second robot to be in position to read the APRILTAG
      if(IRDetected == true){
        chassis.setWheelTargetSpeeds(0,0);
        findRamp();
        robotState = ROBOT_APRIL_TAG;
      }
      //if the IR button has NOT been detected and we are too close to an object we make a right turn, setting a timer, and we update our direction
      else{
        robotState = ROBOT_TURNING;
        updateDirection();
        chassis.setWheelTargetSpeeds(8, -8);
        setTimer(1600);
      }
    }
    //if an object is far away
    else{
      if(IRDetected == false){
        //if the IR button has NOT been detected, we make a right turn to check if the button is on the robot's right, if we haven't checked yet, setting a flag "checkIRrecovery" to true, and we switch to the TURNING state
        if(checkIRrecovery == false){
          checkIRrecovery = true;
          robotState = ROBOT_TURNING;
          chassis.setWheelTargetSpeeds(8, -8);
          setTimer(1600);
        }
        //if the IR button has been detected and we already checked the robot's right, we go back in NAVIGATION
        else{
          robotState = ROBOT_NAVIGATION;
          checkIRrecovery = false;
        }
        }
        //if the IR button has been detected and it's far away we just go towards it switching to the NAVIGATION state
      else{
        robotState = ROBOT_NAVIGATION;
      }
    }
  }
}


//-------------------------------------------------------------------------------
//TIMER FUNCTIONS

uint32_t stopTime;

//we set a timer setting the stopTime variable to a duration
void setTimer(uint32_t duration)
{  
  stopTime = millis() + duration;
}

//we check if a timer expired checking if we passed the duration: currTime >= stopTime
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

//if a timer has expired we handle it depending on the state in which we are in
void handleTimerExpired(){

  //if we are in the CENTERING state, once the timer expired it means that we centered on the intersection and we switch to the WAITING state, waiting for a distance reading or an IR camera reading
  if(robotState == ROBOT_CENTERING){
    robotState = ROBOT_WAITING;
  }

   //if we are in the TURNING state, once the timer expired it means that we are done turning, so we check if the IR button has been detected
  else if(robotState == ROBOT_TURNING){

    if(checkIREmitter()){handleIREmitter();}

    //if we were checking the robot's right but we didn't find the IR button, we make a left turn to recover, switching to the CENTERING state to then go back navigating
    if(checkIRrecovery == true){
      if(IRDetected == false){
        robotState = ROBOT_CENTERING;
        chassis.setWheelTargetSpeeds(-8, 8);
        setTimer(1600);
      }
      //if we were checking the robot's right and we found the IR button, we update the direction, we send to the second robot where the ramp is, we set the values for the secret code and we stop,
      //switching to the APRIL_TAG state, waiting for the second robot to be in position to read the APRILTAG
      else{
        updateDirection();
        chassis.setWheelTargetSpeeds(0,0);
        findRamp();
        robotState = ROBOT_APRIL_TAG;
      }
    }
    //if we weren't checking the robot's right, but we were just turning because we were in front of an object we switch to the WAITING state
    else{
      robotState = ROBOT_WAITING;
    }
  }
  //if we are in the PUSHING_BUTTON state, once the timer expired it means that we pushed the button diplaying the APRILTAG, so we send to the third robot the secret code sending it to the MQTT broker.
  //after sending the secret code we stop and we go in the IDLE state
  else if(robotState == ROBOT_PUSHING_BUTTON){

    idle();
    robotState = ROBOT_IDLE;
  }
}


//-----------------------------------------------------------------------------------------------
// DETECTING IR BUTTON

String openParen = ("(");
String comma = (",");
String closeParen = (")");
String printString;

//we check if we have any new available readings from the IR Positioning Camera
bool checkIREmitter(){
  static uint32_t lastIRread = 0;
  if(millis() - lastIRread > 50)
  {
    irFinder.requestPosition();
    lastIRread = millis();
  } 
  if(irFinder.available()){
    return true;
  }
  else{
    return false;
  }
}

//if we have any new readings that are not empty, that are different than 1023, we set the flag IRDetected to true
void handleIREmitter(){
  int i = 0;
  Point point = irFinder.ReadPoint(i);
  if(point.x != 1023 && point.y != 1023){
    IRDetected = true;
  }
}

//-----------------------------------------------------------------------------------------------
// MQTT COMMUNICATION WITH SECOND ROBOT WITH OpenMV CAMERA

//variable used to send the column of the ramp to the second robot in charge of reading the APRILTAG
int rampX;

//function to send a message to the MQTT broker
void sendMessage(const String& topic, const String& message) {
  Serial1.println(topic + String(':') + message);
}

//variable used to store the messages form the broker
String serString1;

//function used to check if there's any message avaible in the Serial1, in the broker
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

//function used to check if a message with the topic onPlatform has been sent to the MQTT broker, if it has been sent it means that the second robot is in position to read the APRILTAG
bool receiveOnPlatform(){

  bool returnVal = false;
  String searchingTopic = ("onPlatform");
  String recieveTopic = serString1.substring(0, serString1.indexOf(':'));

  int startMessage = serString1.indexOf(':') + 1;

  if(searchingTopic.equals(recieveTopic)){
    returnVal = true;
    //Serial.println(recieveTopic);
    String recieveMessage = serString1.substring(startMessage);
    Serial.println(recieveMessage);
  }
  else{
    returnVal = false;
  }

  serString1 = "";
  return returnVal;
}

//if onPlatform has been sent and the second robot is in position to read the APRILTAG, we drive forward for 2.1 seconds to push the button to display the tag, and we switch to the PUSHING_BUTTON state
void handleSecondRobotOnPlatform(){
  chassis.setWheelTargetSpeeds(8, 8);
  setTimer(2100);
  robotState = ROBOT_PUSHING_BUTTON;
}


//function used to find the column of the ramp and to send it to the second robot
void findRamp(){
  if(xCoord == 0 && currentDirection == 1){
    rampX = 1;
  }
  else if(xCoord == 1 && currentDirection == 2){
    rampX = 1;
  }
  else if(xCoord == 2 && currentDirection == 3){
    rampX = 1;
  }
  else if(xCoord == 2 && currentDirection == 2){
    rampX = 2;
  }

  String printString = (String)rampX;
  sendMessage("rampX", printString);
}


//-----------------------------------------------------------------------------------------------
// NAVIGATION STATE MACHINE

void navigating()
{
    switch (robotState)
    {

    //in the NAVIGATION state we are following the line and we keep checking if an intersection has been detected
    case ROBOT_NAVIGATION:{
      line_following();
      if(checkForIntersection()){ handleIntersection();}
      break;
    }

    // enter if intersection is found or if we are recovering from a right check turn making a left turn. 
    //We check if the timer has expired, if it has, it means that we centered on the intersection or that we are done turning, and we switch to the WAITING state
    case ROBOT_CENTERING:{
      if(checkTimerExpired()){
        handleTimerExpired();
      } 
      break;
    }

    //enter if we are making a right turn, if the timer expires it means that we are done turning and we swtch to the WAITING state
    case ROBOT_TURNING:{
      if(checkTimerExpired()){
        handleTimerExpired();
      } 
      break;
    }

    //enter any time we are done turning or centering, we check for the IR button and we check for a new distance reading
    case ROBOT_WAITING:{
      if(checkIREmitter()) {handleIREmitter();}
      float distanceReading=0;
      bool hasNewReading = hc_sr04.getDistance(distanceReading);
      if(hasNewReading){
        handleNewDistanceReading(distanceReading);
      }
      break;
    }

    //enter if we detected the IR button. In this state we are just waiting in front of the button for a message from the second robot saying that it's in position to read the APRILTAG
    //if it is in the position and the message has been sent, we push the button switching to the PUSHING_BUTTON state
    case ROBOT_APRIL_TAG:{
      if(checkSerial1()){
        if(receiveOnPlatform()){
          handleSecondRobotOnPlatform();
        }
      }
      break;
    }
      
    //enter if we are pushing the button. We check if the timer has expired and if it has, we send the secret code to the broker and we stop.
    case ROBOT_PUSHING_BUTTON:{
      if(checkTimerExpired()){
        handleTimerExpired();
      } 
      break;
    }
    
    default : break;
  }
}