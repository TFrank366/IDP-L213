// the main program file for the arduino
#include <Servo.h>
#include <Adafruit_MotorShield.h>
#include <WiFiNINA.h>
#include <Arduino_LSM6DS3.h>
#include "utils.h"
#include "movement.h"

// pin for each component =======================================================================
// digital pin 2 is borked
const int oLedPin =         4; // Protoboard configuration, do not change unnecessarily
const int rLedPin =         3; // Protoboard configuration, do not change unnecessarily
const int gLedPin =         5; // Protoboard configuration, do not change unnecessarily
const int armServoPin =     9; // check again
const int clawServoPin =   10; // check again
const int distSensorPin =   7; // Protoboard configuration, do not change unnecessarily

// analogue pins
// analogue pin 4 is messed up - AVOID
// dont you dare touch pin 5 or ill mess you up
// line sensor
const int rightSensorPin =  A0;  // Protoboard configuration, do not change unnecessarily
const int leftSensorPin =   A1;  // Protoboard configuration, do not change unnecessarily
// colour sensor
const int bLDRPin =         A3;  // Protoboard configuration, do not change unnecessarily
const int rLDRPin =         A2;  // Protoboard configuration, do not change unnecessarily

// ==============================================================================================
const int fSpeed =         200; // motor speed for general movement (see movement.cpp) ================================================================ changes based on the weight
// ==============================================================================================
float kp =                 1.8; // kp for turning
// ==============================================================================================

unsigned long prevOLedChangeMillis = 0;
unsigned long prevMillis = 0;
unsigned long currMillis = 0;
unsigned long changeMillis = 0;
unsigned long oLedInterval = 500; // for 2Hz flashing of oLed


// set with reference from overhead camera for precision turning and forward movement
float angleError = 0;
float distanceError = 0;

// keep a track of program variables
int lastLineVals[2] = {0, 0};
int currentCrossingCount = 0;


programStageName currentStage;
int programIteration = 0;
bool stageShouldAdvance = false;
// Flags to indicate reversing or stopping
bool programHalted = true; // to indicate if the program is running
bool turnNotStarted = true;
bool angleReceived = false;

// Timer to check how long program has lasted for
unsigned long lastMillis = 0;

// enum to run a certain code when BLUE or RED is activated
enum Color {BLUE, RED};

// Structs for Optosensors and LEDs
struct Sensor {
  int pin;
  int value;
};

struct Led {
  int pin;
  bool state;
};

Logger l((unsigned long)0, BOTH);

bool MOTORSREVERSED = true; // in case motor direction flipped, then just invert this flag

// Connects to motor shield to command wheel movement
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(4);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(3);

// connects to servos to command servo movement
Servo armServo;
int armServoUp = 20; // 20 is up
int armServoDown = 130; // 130 is fully down

Servo clawServo;
int clawServoClosed = 162;// 162 is fully closed
int clawServoOpen = 105;// 105 is open

// create led structs
Led oLed;
Led rLed;
Led gLed;

//create sensor structs
//optoswitches
Sensor rightSensor;
Sensor leftSensor;
Sensor distSensor;
//LDRs
Sensor rLDR;
Sensor bLDR;

// global flag to keep track of if the motors are running to know when to flash oLed
bool motorsActive = false;

// handles line following
//Movement::FollowLine* lineFollower(210, 35, (unsigned long)100);

void setup() {
  // setup serial link and bluetooth
  Serial.begin(9600);
  Serial.println("Serial up");

  // initialise the motors
  AFMS.begin();
  Serial.println("Motor shield up");

  pinMode(NINA_RESETN, OUTPUT);
  digitalWrite(NINA_RESETN, LOW);
  SerialNina.begin(115200);

  // initialise IMU for gyroscope
  IMU.begin();
  Serial.println("IMU up");

  // Startup phrase
  Serial.println("hello world");

  //initialise led structs
  pinMode(oLedPin, OUTPUT);
  oLed = {.pin = oLedPin};
  pinMode(rLedPin, OUTPUT);
  rLed = {.pin = rLedPin};
  pinMode(gLedPin, OUTPUT);
  gLed = {.pin = gLedPin};
  Serial.println("leds setup");

  // initialise the sensors
  pinMode(rightSensorPin, INPUT);
  rightSensor.pin = rightSensorPin;
  pinMode(leftSensorPin, INPUT);
  leftSensor.pin = leftSensorPin;
  pinMode(distSensorPin, INPUT);
  distSensor.pin = distSensorPin;
  pinMode(bLDRPin, INPUT);
  bLDR.pin = bLDRPin;
  pinMode(rLDRPin, INPUT);
  rLDR.pin = rLDRPin;
  Serial.println("sensors setup");

  armServo.attach(armServoPin);
  clawServo.attach(clawServoPin);

  angleError = 180; // this will be the value that we need to actually shift the robot by. Note that this can be a signed angle to indicate direction!
  currentStage = START; // testing from the beginning
}

// general function to apply a motorSetting struct onto the motors
void setMotors(Movement::MotorSetting mSetting) {
  leftMotor->setSpeed(min(mSetting.speeds[0], 255));
  rightMotor->setSpeed(min(mSetting.speeds[1], 255));
  
  // here to correct the direction of movement in case the motors are connected backwards
  if (!MOTORSREVERSED) { // ie supposed to be going forward
    leftMotor->run(mSetting.directions[0]);
    rightMotor->run(mSetting.directions[1]);
  }
  else { // ie supposed to be going backward
    if (mSetting.directions[0] == FORWARD) {
      leftMotor->run(BACKWARD);
    } else {
      leftMotor->run(FORWARD);
    }
    if (mSetting.directions[1] == FORWARD) {
      rightMotor->run(BACKWARD);
    } else {
      rightMotor->run(FORWARD);
    }
  }

  if ((mSetting.speeds[0] > 0 || mSetting.speeds[1] > 0 ) && !motorsActive) { // We have asked it to start moving but the motorsActive flag is still true
    motorsActive = true;
  } else if (mSetting.speeds[0] == 0 && mSetting.speeds[1] == 0 && motorsActive) { // We have asked it to stop moving motorsActive flag is still false
    motorsActive = false;
    //programHalted = true;
    digitalWrite(oLed.pin, false);
  }
}

float gx, gy, gz;
unsigned long nowMillis;
unsigned long crossings_seen_time;
bool all_crossings_seen = false;
float angleDelta;
bool grabStarted = false;
int clawPos;

bool checkForCrossing(int lineVal) { // must have last 2 values as a crossing for it to confirm that it was in fact a crossing, and add one to the crossing counter
  if (lastLineVals[0] != 0b11 && lastLineVals[1] != 0b11 && lineVal == 0b11) {
    lastLineVals[1] = lastLineVals[0];
    lastLineVals[0] = lineVal;
    return true;
  } else {
    lastLineVals[1] = lastLineVals[0];
    lastLineVals[0] = lineVal;
    return false;
  }
}

Movement::MotorSetting getMovementFromStage(programStageName stageName, int lineVal) {
//  Serial.print("stage: ");
//  Serial.println(stageName);
// leave the case clauses in this order  (10 March update: changing to the actual order of stages for ease of reading, hopefully this does not change anything)
 
  switch (stageName) {

    case GRAB_BLOCK: // had to change order because it works better, somehow =======================================================================================================================
      l.logln("block grabbed");
      stageShouldAdvance = true; // here for now
//      if (!grabStarted) {
//        clawServo.write(clawServoOpen);
//        clawPos = clawServoOpen;
//        grabStarted = true;
//      } else if (clawPos >= clawServoClosed){
//        stageShouldAdvance = true;
//        return Movement::getMovement(Movement::STOP, 0, 0);
//      } else {
//        clawPos += 1;
//        clawServo.write(clawPos);
//      }
//      return Movement::getMovement(Movement::STRAIGHT, 90, 0);
      break;

      case RAISE_BLOCK: // ==========================================================================================================================================================
        l.logln("block raised");
        stageShouldAdvance = true;
          // raise arm servo in a loop
//          return Movement::getMovement(Movement::STOP, 0, 0);
//          for (int i = armServoDown; i < armServoUp; i++) {
//            armServo.write(i);
//          }
          break;

    case MOVE_TO_LINE_FROM_BLOCK: // ==========================================================================================================================================================
        l.logln("moving back to the line");
        stageShouldAdvance = true;
        break;
        
    case LONG_TRAVERSE_1: // ==========================================================================================================================================================
      l.logln("line follow the second time!");
      // check is we've hit a crossing
      if (checkForCrossing(lineVal)) {
        currentCrossingCount++;
        l.logln("lf crossing");
      }
      if (currentCrossingCount == 1) {
        if (!all_crossings_seen) {
          crossings_seen_time = currMillis;
          all_crossings_seen = true;
        }
        if (currMillis - crossings_seen_time > 2000) {
          stageShouldAdvance = true;
        }
        return Movement::getMovement(Movement::LINE_FOLLOW, fSpeed, lineVal);
      } else {
        return Movement::getMovement(Movement::LINE_FOLLOW, fSpeed, lineVal);
      }
      break;


    case START: // ==========================================================================================================================================================
      // raise the grabber to the highest point
      armServo.write(armServoUp);
      if (checkForCrossing(lineVal)) {
        currentCrossingCount++;
        l.logln("start crossing");
      }
      
      if (currentCrossingCount == 2 || programIteration > 0) {
        stageShouldAdvance = true;
        return Movement::getMovement(Movement::STOP, 0, 0);
      } else {
        return Movement::getMovement(Movement::LINE_FOLLOW, fSpeed, lineVal);
      }
      break;

    case LONG_TRAVERSE_0: // ==========================================================================================================================================================
      //Serial.println("line follow");
      // check is we've hit a crossing
      if (checkForCrossing(lineVal)) {
        currentCrossingCount++;
        l.logln("lf crossing");
      }
      if (currentCrossingCount == 1) {
        if (!all_crossings_seen) {
          crossings_seen_time = currMillis;
          all_crossings_seen = true;
        }
        if (currMillis - crossings_seen_time > 2000) {
          stageShouldAdvance = true;
        }
        return Movement::getMovement(Movement::LINE_FOLLOW, fSpeed, lineVal);
      } else {
        return Movement::getMovement(Movement::LINE_FOLLOW, fSpeed, lineVal);
      }
      break;

    case TURN_TO_BLOCK: // ==========================================================================================================================================================
      armServo.write(armServoDown);
      //Serial.println("turning");
      Movement::MotorSetting mSetting;
      nowMillis = millis();
      if (abs(angleError) > 1) { // More than 1 degree or so away, cannot be perfect
        if (turnNotStarted) {
          lastMillis = nowMillis;
          turnNotStarted = false; // keep a track of 
        }
        // get angular velocity
        if (IMU.gyroscopeAvailable() && IMU.readGyroscope(gx, gy, gz)) {
          Serial.println(angleError);
          //l.logln(String(gz + 0.45) + " " + String(angleError));
          // get the angular dispacement since last time
          // might need to set last millis to the current time when a turn is started to avoid large errors
          angleDelta = ((gz + 0.45) / 1000) * (nowMillis - lastMillis) ; // the 0.45 offset is to account for the fact that the suspended vehicle (not rotating) seems to read a -ve value
          // update angle error with dθ
          // set motors to kp*angle error
          if (angleError < 0) {
            mSetting =  Movement::getMovement(Movement::SPIN, min((int)kp * angleError, -90), 0);
          } else {
            mSetting =  Movement::getMovement(Movement::SPIN, max((int)kp * angleError, 90), 0);
          }
          
          angleError -= angleDelta / (0.8*1.05);
          }
        } else {
          stageShouldAdvance = true;
          return Movement::getMovement(Movement::STOP, 0, 0); // stops when angle has been reached
        }
        lastMillis = nowMillis;
        return mSetting;
        break;

      case MOVE_TO_BLOCK: // ==========================================================================================================================================================
      l.logln("move to block");
        if (digitalRead(distSensor.pin) == 0) {
          stageShouldAdvance = true;
          return Movement::getMovement(Movement::STOP, 0, 0);
//        } else if (programIteration == 0){
//          // if block is too far away, keep moving towards it
//          return Movement::getMovement(Movement::LINE_FOLLOW, -70, lineVal);
        } else { // openCV ========================================================================================================================================
         return Movement::getMovement(Movement::STRAIGHT, -70, 0);
        }
        break;
     
     case SENSE_BLOCK_COLOR: // ==========================================================================================================================================================
      Color blockCol = getColorVal(rLDR, bLDR);
      if (blockCol == BLUE) {
        //l.logln("blue");
        digitalWrite(gLed.pin, true);
        delay(5100);                              // change to 5100 for final
        digitalWrite(gLed.pin, false);
      }
      else {
        //l.logln("red");
        digitalWrite(rLed.pin, true);
        delay(5100);                              // change to 5100 for final
        digitalWrite(rLed.pin, false);
      }
      l.logln("block colour got");
      stageShouldAdvance = true;
      break;



  
    case LOWER_BLOCK: // ==========================================================================================================================================================
      // lower arm servo in a loop
      // return Movement::getMovement(Movement::STOP, 0, 0);
      // break;
    
    case MOVE_TO_DROP_ZONE: // ==========================================================================================================================================================

    case DROP_BLOCK: // ==========================================================================================================================================================
      //l.logln("stopped");
      return Movement::getMovement(Movement::STOP, 0, 0);
      break;
    
    case MOVE_TO_LINE_FROM_DROP: // ==========================================================================================================================================================

    default: // ==========================================================================================================================================================
      Serial.println("default");
      return Movement::getMovement(Movement::STOP, 0, 0);
  }
}

void advanceStage() {
  currentCrossingCount = 0;
  turnNotStarted = true;
  angleReceived = false;
  grabStarted = false;
  all_crossings_seen = false;
  stageShouldAdvance = false;
  if (currentStage == MOVE_TO_LINE_FROM_DROP) {
    programIteration++;
  }
  currentStage = static_cast<programStageName>(currentStage + 1);

  // do initial setup for the stages that need it
  switch (currentStage) {
    case TURN_TO_BLOCK:
      if (programIteration == 10) { // set to 0 ==========================
        angleError = 0;
        angleReceived = true;
      } else {
        // send a request 
        l.logln("<angle,block>");
      }
      break;
    case MOVE_TO_LINE_FROM_BLOCK:
      angleError = 180; 
      break;     
  }
  l.logln("stage advanced");
}

int getLineVal(Sensor left, Sensor right) {
  int lineVal = 0;
  lineVal |= analogRead(right.pin) < 25; // change the read values based on line reading ==============================================================
  //delay(10);
  lineVal |= (analogRead(left.pin) < 25) << 1; // change the read values based on line reading ==============================================================
  return lineVal;
}

String getValsString(Sensor left, Sensor right) { // this function is to print out line follower sensor values
  int a  = analogRead(left.pin);
  //delay(10);
  int b  = analogRead(right.pin);
  //delay(10);
  return String(a) + " " + String(b);
}

// gets the colour of what is in front of the colour sensor
// won't be accurate unless distSensor is close enough
Color getColorVal(Sensor rLDR, Sensor bLDR) {
  int bVal = analogRead(bLDR.pin);
  int rVal = analogRead(rLDR.pin);
  if (bVal < rVal) { // Since rVal always seems to be reading more than bVal, change accordingly ==============================================================
    return BLUE;
  } else {
    return RED;
  }
}

String getSerialCommand() {
  String command = Serial.readString();
  command = command.substring(0, command.length());
  return command;
}

// bluetooth command receiver
String getBTSerialCommand() {
  String command = SerialNina.readString();
  return command;
}

// What does every input command to the Arduino do (raises flags that have corresponding actions)
void commandHandler(String command) {
  l.logln("command received");

  if (command == "stop" || command == "stopstop") {
// serial command reciever
    //l.logln("stopping");
    programHalted = true;
  }
  else if (command == "go" || command == "gogo") {
    //l.logln("starting");
    programHalted = false;
  } else if (command.startsWith("<angle,")) {
    l.logln("angle received");
    if (!angleReceived) {
      angleReceived = true;
      angleError = command.substring(7, command.length() - 1).toFloat();
      l.logln(angleError);
    }
    // received angle error
  } else if (command.startsWith("distance")) {
    // received the distance error 
  }
}

void loop() {
  //put your main code here, to run repeatedly: ======================================================================================================================================
  //Serial.println("loop"); // to check if the loop is running
  currMillis = millis();
  changeMillis = currMillis - prevMillis;
  //Serial.println(getValsString(leftSensor, rightSensor)); // print out line sensor values

  if (programHalted) {
    setMotors(getMovement(Movement::STOP, 0, 0));
  } else {
    if (stageShouldAdvance) {
      advanceStage();
    } else {
      int lineVal = getLineVal(leftSensor, rightSensor);
      setMotors(getMovementFromStage(currentStage, lineVal));
    }    
  }

  //  oLed flashes if the motors are active
  if (motorsActive) {
    if (currMillis - prevOLedChangeMillis >= oLedInterval) {
      // save the last time you blinked the LED
      prevOLedChangeMillis = currMillis;
      // set the LED with the ledState of the variable:
      digitalWrite(oLed.pin, !oLed.state);
      oLed.state = !oLed.state;
    }
  }

  // scan for any commands in the BT or USB serial buffers
  if (Serial.available() > 0) {commandHandler(getSerialCommand());}
  if (SerialNina.available() > 0) {commandHandler(getBTSerialCommand());}
  prevMillis = currMillis;
}
