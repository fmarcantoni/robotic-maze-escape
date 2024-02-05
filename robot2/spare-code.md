# robot.cpp

### Timer expires func. from 1st render of lab 4 state machine
```
bool checkTimerExpired(void){ //to be implemented
  bool expired = false;
  if(timer expired) expired = true;
  else expired = false;
  return expired;
}

void handleTimerTimerExpired(void){
  if(navigationState == ROBOT_CENTERING || navigationState == ROBOT_TURNING){
    chassis.setWheelTargetSpeeds(0,0);
    navigationState = ROBOT_WAITING;
  }
}
```

### Handle distance reading func. from 1st render of lab 4 state machine
```
void handleNewDistanceReading(float distanceReading)
{
  float treshold; //distance from next intersection
  bool objectDetected = false;

#ifdef __DEBUG_RANGEFINDER__
    Serial.println(distanceReading);
#endif

     if(distanceReading < treshold){ //if an object is detected, we turn and update direction
      objectDetected = true;

      chassis.setWheelTargetSpeeds(107, -107);
      navigationState = 0;
      currentDirection = updateDirection(currentDirection);
     }
     else{ //if an object is NOT detected, we keep navigating
      objectDetected = false;


      chassis.setWheelTargetSpeeds(107, 107);
      //line_following();
      navigationState = 1;
     }
  return objectDetected;
}
```

### States from rend 1 lab 4 state machine
```
enum NAVIGATION_STATE{
  ROBOT_DRIVING,
  ROBOT_CENTERING,
  ROBOT_TURNING,
  ROBOT_WAITING,
};

NAVIGATION_STATE navigationState = ROBOT_DRIVING;
```
### Process distance sensors
```
void processDistanceSensors(void)
{
    /** Check the distance sensor.
     * We return true only if there is a new reading, which is passed by reference.
     * It hardly needs to be done this way, but passing by reference is a useful tool,
     * so we start with a 'lightweight' example here.
     */
    float distanceReading = 0;
    bool hasNewReading = hc_sr04.getDistance(distanceReading);
    //if(hasNewReading) handleNewDistanceReading(distanceReading);
}
```
### Distance handler for hysterisis function from I don't remember which lab
```
void handleNewDistanceReading(float distanceReading)
{
#ifdef __DEBUG_RANGEFINDER__
    Serial.println(distanceReading);
#endif
    if(robotState == ROBOT_STANDOFF)
    {
      addHysterisis(distanceReading);
      Serial.println(distanceReading);
    }
}
```

### All Standoff code
```
void stateDef(float distanceReading, float runSpeed, float approachSpeed){

    float distanceDiff = distanceReading - targetDist;

    if (distanceDiff < -0.5 ){
      chassis.setWheelTargetSpeeds(runSpeed, runSpeed);
      float targetDist = 25;
      }
    else if (distanceReading - targetDist > 0.5) {
      chassis.setWheelTargetSpeeds(approachSpeed, approachSpeed);
      targetDist = 25;
    }
    else targetDist = 40;
 }
void addHysterisis(float distanceReading){

  float prevErrorA, prevErrorR;

  float Kp = 1;
  float Kd = 0.2;

  float errorAppSpeed = (distanceReading - 25);
  float errorRunSpeed = (distanceReading - 40);

  float diffErrorA =  errorAppSpeed - prevErrorA;
  float diffErrorR = errorRunSpeed - prevErrorR;

  float approachSpeed = Kp * errorAppSpeed + diffErrorA * Kd; //input speed approach
  float runSpeed = Kp * errorRunSpeed + diffErrorR * Kd; //input reverse speed

  prevErrorA = errorAppSpeed;
  prevErrorR = errorRunSpeed;

  //constrain(approachSpeed, 0, 20);
  //constrain(runSpeed, -50, 0);

  Serial.println(distanceReading);

  stateDef(distanceReading, runSpeed, approachSpeed);
}
```

# chassis.cpp

### Angle printer from lab 3
```
    Serial.print(predictedAngle);
    Serial.print(' ');
    Serial.print(ObservedPitchAngle);
    Serial.print(' ');
    Serial.print(filteredAngle);
    Serial.print(' ');
    Serial.print(gyroBias);    

    Serial.print('\n');
```


### Old handle intersection

void handleIntersection(void)
{
  if (checkForIntersection() == 1)
  {
    unsigned long previousMillis = millis();
    while(true)
    {
  //previousMillis = currentMillis;
  unsigned long currentMillis = millis();
  float i = (currentMillis - previousMillis) / 1000;
  Serial.print("currentMillis - previousMillis:");
  Serial.print(currentMillis - previousMillis);
  Serial.print(" ");
  Serial.print(" ");
  Serial.print("i:");
  Serial.print(i);
  Serial.println();

  if(i <= 10)
  {
  //float t = 5 - (i/2);
  chassis.setMotorEfforts(((40/3)/(2*i)), (24/(2*i))); //when we are at an intersection we stop (if we are actually right on the intersection and we don't need to drive forward)
  Serial.println("t < 10");
  logLocation();
  }
  else{
    chassis.setMotorEfforts(0,0);

  }
    }
  }

   else if(i > 10 && i <= 11.8)
  {
    chassis.setMotorEfforts((3*i), (-5*i));
    Serial.println("10 < i < 13");
    logLocation();
  }
  else if(i > 13)
  {
    chassis.setMotorEfforts(0, 0);
    Serial.println("End Turn");
    logLocation();
    Serial.println("break");
    break;
  }
    }
  }
  else
  {

  }

}

*/


### Old State Machine
#define INTERSECTION 0
#define OBJECT 1
#define DEFAULT 2


int state = 2;

void navigating()
{

  if(checkForIntersection()){handleIntersection(); state = INTERSECTION;}


  if(checkForObject())
  {handleTurn(); 
  state = OBJECT;}
  else state = DEFAULT;

  //if (chassis.imu.getStatus() & 0x01)
  //{
    switch (state)
    {
    case INTERSECTION: // enter if intersection is found
      if(checkCreepingTimerExpired()) handleCreepingTimerExpired();
      Serial.println("INTERSECTION");

      if(checkForObject()){handleTurn(); state = OBJECT;}
      else{state = DEFAULT;}
      break;

    case OBJECT:
      Serial.println("OBJECT DETECTED");
      //handleTurn();
      if(checkTurningTimerExpired()) handleTurningTimerExpired();
      break;

    case DEFAULT:
      line_following();
      break;
    }
  }