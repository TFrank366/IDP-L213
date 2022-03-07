// main.cpp
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
const int servo1Pin =       9; // check again
const int servo2Pin =      10; // check again

// analogue pins
// analogue pin 4 is messed up - AVOID
// line sensor
const int rightSensorPin =  A0;  // Protoboard configuration, do not change unnecessarily
const int leftSensorPin =   A5;  // Protoboard configuration, do not change unnecessarily
// colour sensor
const int bLDRPin =         A3;  // Protoboard configuration, do not change unnecessarily
const int rLDRPin =         A2;  // Protoboard configuration, do not change unnecessarily
const int distSensorPin =   A1;  // Protoboard configuration, do not change unnecessarily
// ==============================================================================================
const int fSpeed =         210; // motor speed for general movement (see movement.cpp)
// ==============================================================================================
float kp =                 1.8; // kp for turning
// ==============================================================================================

// enum to run a certain code when BLUE or RED is activated
enum Color {BLUE, RED}; 

// set with reference from overhead camera for precision turning and forward movement
float angleError = 0;
float distanceError = 0;

// Flags to indicate reversing or stopping
bool MOTORSREVERSED = false;
bool robotStopped = true;
bool turnNotStarted = true;

// Timer to check how long program has lasted for 
unsigned long lastMillis = 0;

// Structs for Optosensors and LEDs
struct Sensor {
  int pin;
  int value;
  };

struct Led {
  int pin;
  bool state;
  };

Logger l((unsigned long)200, BOTH);

// Connects to motor shield to command wheel movement
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(3);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(4);

// connects to servos to command servo movement
Servo servo;

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
Movement::FollowLine* lineFollower(fSpeed, 35, (unsigned long)100);

void setup() {
  // setup serial link and bluetooth 
  Serial.begin(9600);
  pinMode(NINA_RESETN, OUTPUT);         
  digitalWrite(NINA_RESETN, LOW);
  SerialNina.begin(115200);
  
  // initialise IMU for gyroscope
  IMU.begin();

  // Startup phrase
  Serial.println("hello world");

  //initialise led structs
  pinMode(oLedPin, OUTPUT);
  oLed = {.pin = oLedPin};
  pinMode(rLedPin, OUTPUT);
  rLed = {.pin = rLedPin};
  pinMode(gLedPin, OUTPUT);
  gLed = {.pin = gLedPin};

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

  // initialise the motors
  AFMS.begin();

  float actual_angle_error = 360; // this will be the value that we need to actually shift the robot by. Note that this can be a signed angle to indicate direction!
  angleError = actual_angle_error*0.8*1.1; // change these multipliers if necessary, since gyro does not have perfect accuracy re angular velocity
  
}

// general function to apply a motorSetting struct onto the motors
void setMotors(Movement::MotorSetting mSetting) {
  //l.logln("motors set");
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
  } 
  
  else if (mSetting.speeds[0] == 0 && mSetting.speeds[1] == 0 && motorsActive) { // We have asked it to stop moving motorsActive flag is still false
    motorsActive = false;
    robotStopped = true;
    digitalWrite(oLed.pin, false);
  }
}


Movement::MotorSetting getMovementFromStage(programStageName stageName, int lineVal) {
  switch (stageName) {
      case START:
        return Movement::getMovement(Movement::STRAIGHT, 70);
        break;
      case MOVE_TO_BLOCK:
        // if block is too far away, keep moving towards it
        return Movement::getMovement(Movement::STRAIGHT, 70);
        break;
      // case for grabbing the block, need to move forward slow
      case GRAB_BLOCK:
        return Movement::getMovement(Movement::STRAIGHT, 70);
        break;
        
      case LONG_TRAVERSE_0:
      case LONG_TRAVERSE_1:
        return lineFollower->getMotorSetting(lineVal);
        break;
        
      case SENSE_BLOCK_COLOR:
      case RAISE_BLOCK:
      case LOWER_BLOCK:
      case DROP_BLOCK:
        //l.logln("stopped");
        return Movement::getMovement(Movement::STOP, 0);
        break;

      case TURN_TO_BLOCK:
      case MOVE_TO_LINE_FROM_BLOCK:
      case MOVE_TO_DROP_ZONE:
      case MOVE_TO_LINE_FROM_DROP:
        Movement::MotorSetting mSetting;
        unsigned long nowMillis = millis();
        if (abs(angleError) > 1) { // More than 1 degree or so away, cannot be perfect
          if (turnNotStarted) {
            lastMillis = nowMillis;
            turnNotStarted = false; /////////////////// keep a track of!! =============================================================================================
          }
          float gx, gy, gz;
          
          // get angular velocity
          if (IMU.gyroscopeAvailable() && IMU.readGyroscope(gx, gy, gz)) {
            l.logln(String(gz + 0.45) + " " + String(angleError));
            // get the angular dispacement since last time
            // might need to set last millis to the current time when a turn is started to avoid large errors
            float angleDelta = (gz + 0.45)*(nowMillis - lastMillis)/1000;
            // update angle error with dθ
            // set motors to kp*angle error
            if (angleError < 0) {
              mSetting =  Movement::getMovement(Movement::SPIN, min((int)kp*angleError, -80));
            } 
            else {
              mSetting =  Movement::getMovement(Movement::SPIN, max((int)kp*angleError, 80));
            }
            angleError -= angleDelta;
          }
        } 
        else {
          return Movement::getMovement(Movement::STOP, 0); // stops when angle has been reached 
        }

//        else if dist error > threshold
//        else
//        end stage as they are small enough

//        lastMillis = nowMillis;
//
//        return mSetting;
//        break;
//      default:
//        return Movement::getMovement(Movement::STOP, 0);
    }
}


int getLineVal(Sensor s1, Sensor s2) {
  int lineVal = 0;
  lineVal |= analogRead(s1.pin) < 25; // change the read values based on line reading
  delay(10);
  lineVal |= (analogRead(s2.pin) < 25) << 1; // change the read values based on line reading
  return lineVal;
}

// gets the colour of what is in front of the colour sensor
// won't be accurate unless distSensor reads < 300

Color getColorVal(Sensor rLDR, Sensor bLDR) {
  int bVal = analogRead(bLDR.pin);
  int rVal = analogRead(rLDR.pin);
  if (bVal < rVal - 200){ // Since rVal always seems to be reading more than bVal, change accordingly ==============================================================
    return BLUE;
  } 
  else {
    return RED;  
  }
}

String getValsString(Sensor s1, Sensor s2) { // this function is to print out line follower sensor values
  int a  = analogRead(s2.pin);
  delay(10);
  int b  = analogRead(s1.pin);
  delay(10);
  return String(a) + " " + String(b);
}

// serial command reciever
String getSerialCommand() {
  String command = Serial.readString();
  command = command.substring(0, command.length());
  //command.trim();
  return command;
} 

// bluetooth command receiver
String getBTSerialCommand() {
  String command = SerialNina.readString();
  //command = command.substring(0, command.length()/2);
  return command;
}

// What does every input command to the Arduino do (raises flags that have corresponding actions)
void commandHandler(Movement::FollowLine* lineFollowerPtr, String command) {
  l.logln("command received");
  
  if (command == "stop" || command == "stopstop") {
    l.logln("stopping");
    robotStopped = true;
  } 
  else if (command == "go" || command == "gogo") {
    l.logln("starting");
    robotStopped = false;
  }
}

unsigned long previousMillis = 0;
unsigned long oLedInterval = 500; // for 2Hz flashing of oLed

void loop() {
  //put your main code here, to run repeatedly:
  //Serial.println("loop"); // to check if the loop is running
  unsigned long currentMillis = millis();

  //servo.write(min(180, int(currentMillis/100)));

  // the main movement code 
  if (robotStopped) {
    setMotors(Movement::getMovement(Movement::STOP, 0));
    //l.logln(analogRead(distSensor.pin));
    if (analogRead(distSensor.pin) < 800) { // change based on an appropriate distance we can get to the block
      delay(10);
      l.logln(String(analogRead(rLDR.pin)) + " " + String(analogRead(bLDR.pin)));
      delay(10);
      Color blockCol = getColorVal(rLDR, bLDR);
      if (blockCol == BLUE) {
        l.logln("blue");
        digitalWrite(gLed.pin, true);
        delay(100);                              // change to 5100 for final
        digitalWrite(gLed.pin, false);
      } 
      else {
        l.logln("red");
        digitalWrite(rLed.pin, true);
        delay(100);                              // change to 5100 for final
        digitalWrite(rLed.pin, false);
      }
    }
    
  } 
  else {
    int lineVal = getLineVal(rightSensor, leftSensor); 
    l.logln(String(analogRead(leftSensor.pin)) + " " + String(analogRead(rightSensor.pin))); // just print out 
//    setMotors(lineFollower->getMotorSetting(lineVal));
//    setMotors((Movement::MotorSetting){.speeds = {fSpeed, fSpeed}, .directions = {FORWARD, FORWARD}});
//    setMotors(getMovementFromStage(currentStage, lineVal));
//    setMotors(getMovementFromStage(LONG_TRAVERSE_0, lineVal));
    }

//  oLed flashes if the motors are active
  if (motorsActive) {
    if (currentMillis - previousMillis >= oLedInterval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;
      // set the LED with the ledState of the variable:
      digitalWrite(oLed.pin, !oLed.state);
      oLed.state = !oLed.state;
    }
  }

  // scan for any commands in the BT or USB serial buffers
  if (Serial.available() > 0) {commandHandler(lineFollower, getSerialCommand());}
  if (SerialNina.available() > 0) {commandHandler(lineFollower, getBTSerialCommand());}

}
