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
const int oLedPin =         5;
const int rLedPin =         3;
const int gLedPin =         4;
const int servo1Pin =       9;
const int servo2Pin =      10;

// analogue pins
// analogue pin 4 is messed up - AVOID
// line sensor
const int rightSensorPin =  A1;  // analogue pin #
const int leftSensorPin =   A0;  // analogue pin #
// colour sensor
const int bLDRPin =         A3;  // Blue colour LDR voltage (goes down with more light)       
const int rLDRPin =         A5;  // Red colour LDR voltage                                    
const int distSensorPin =   A2;  // OPB704 Voltage (goes down with decreasing distance)
// ==============================================================================================
const int fSpeed =         210; // motor speed for general movement
// ==============================================================================================
float kp =                 1.8; // kp for turning
// ==============================================================================================

enum Color {BLUE, RED};

// set with reference from overhead camera for precision turning and forward movement
float angleError = 0;
float distanceError = 0;

bool MOTORSREVERSED = false;
bool robotStopped = true;

unsigned long lastMillis = 0;
bool turnNotStarted = true;
programStageName currentStage = TURN_TO_BLOCK;



struct Sensor {
  int pin;
  int value;
};

struct Led {
  int pin;
  bool state;
};

Logger l((unsigned long)200, BOTH);

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(3);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(4);

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
//int currSpeed = 0; // keep a track of the current speed
// create object for handling the different movement regimes
// handles line following
Movement::FollowLine* lineFollower(fSpeed, 35, (unsigned long)100);

void setup() {
  // setup serial link
  Serial.begin(9600);
  IMU.begin();
  
  pinMode(NINA_RESETN, OUTPUT);         
  digitalWrite(NINA_RESETN, LOW);
  SerialNina.begin(115200);

  //SerialNina.write(SerialNina.read());
  Serial.println("hello world");

  //init led structs
  pinMode(oLedPin, OUTPUT);
  oLed = {.pin = oLedPin};
  pinMode(rLedPin, OUTPUT);
  rLed = {.pin = rLedPin};
  pinMode(gLedPin, OUTPUT);
  gLed = {.pin = gLedPin};

  // init the sensors
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

  digitalWrite(gLed.pin, true);
  digitalWrite(oLed.pin, true);
  digitalWrite(rLed.pin, true);

  angleError = 360 *0.8*1.1; // change these mutlipliers based on mass difference
  
}


// general function to apply a motorSetting struct onto the motors
void setMotors(Movement::MotorSetting mSetting) {
  //l.logln("motors set");
  Serial.print(mSetting.speeds[0]);
  Serial.print(" ");
  Serial.println(mSetting.speeds[1]);
  leftMotor->setSpeed(min(mSetting.speeds[0], 255));
  rightMotor->setSpeed(min(mSetting.speeds[1], 255));
  
  // handle if the motors are reversed so forward -> backward
  if (!MOTORSREVERSED) {
    leftMotor->run(mSetting.directions[0]);
    rightMotor->run(mSetting.directions[1]);
  } else {
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
  
  if ((mSetting.speeds[0] > 0 || mSetting.speeds[1] > 0 ) && !motorsActive) {
    motorsActive = true;
  } else if (mSetting.speeds[0] == 0 && mSetting.speeds[1] == 0 && motorsActive) {
    motorsActive = false;
    robotStopped = true;
    //digitalWrite(oLed.pin, false);
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
        if (abs(angleError) > 1) {
          if (turnNotStarted) {
            lastMillis = nowMillis;
            turnNotStarted = false; /////////////////// keep a track of!! ===============================
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
            } else {
              mSetting =  Movement::getMovement(Movement::SPIN, max((int)kp*angleError, 80));
            }
            angleError -= angleDelta;
          }
        } else {
          return Movement::getMovement(Movement::STOP, 0);
        }
        // else if dist error > threshold

        // else
        //   end stage as they are small enough

        lastMillis = nowMillis;

        return mSetting;
        break;
      default:
        return Movement::getMovement(Movement::STOP, 0);
    }
}



//                           2  1
// returns an int with bits (s3 s1)
// line on left => 100 => 4
int getLineVal(Sensor s1, Sensor s3) {
  int lineVal = 0;
  lineVal |= analogRead(s1.pin) < 25;
  delay(10);
  lineVal |= (analogRead(s3.pin) < 25) << 1;
  return lineVal;
}

// gets the colour of what is in front of the colour sensor
// won't be accurate unless depth sensor (distSensor) reads < 300
Color getColorVal(Sensor rLDR, Sensor bLDR) {
  int bVal = analogRead(bLDR.pin);
  int rVal = analogRead(rLDR.pin);
  if (bVal < rVal - 200){
    return BLUE;
  } else {
    return RED;  
  }
}

String getValsString(Sensor s1, Sensor s3) {
  int a  = analogRead(s3.pin);
  delay(10);
  int c  = analogRead(s1.pin);
  delay(10);
  return String(a) + " " + String(c);
}

// serial command reciever
String getSerialCommand() {
  String command = Serial.readString();
  command = command.substring(0, command.length());
  //command.trim();
  return command;
} 

String getBTSerialCommand() {
  String command = SerialNina.readString();
  //command = command.substring(0, command.length()/2);
  return command;
}

void commandHandler(Movement::FollowLine* lineFollowerPtr, String command) {
  // print the command string that was received
  //l.logln(command);
  l.logln("command received");
  
  if (command == "stop" || command == "stopstop") {
    l.logln("stopping");
    robotStopped = true;
  } else if (command == "go" || command == "gogo") {
    l.logln("starting");
    robotStopped = false;
  }
}


unsigned long previousMillis = 0;
unsigned long oLedInterval = 500;

void loop() {
  Serial.println("loop");
  //put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();

  //servo.write(min(180, int(currentMillis/100)));

  // the main movement code 
  if (robotStopped) {
    setMotors(Movement::getMovement(Movement::STOP, 0));
    //l.logln(analogRead(distSensor.pin));
    if (analogRead(distSensor.pin) < 800) {
      delay(10);
      l.logln(String(analogRead(rLDR.pin)) + " " + String(analogRead(bLDR.pin)));
      delay(10);
      Color blockCol = getColorVal(rLDR, bLDR);
//      if (blockCol == BLUE) {
//        l.logln("blue");
//        digitalWrite(gLed.pin, true);
//        delay(100);                              // change to 5100
//        digitalWrite(gLed.pin, false);
//      } else {
//        l.logln("red");
//        digitalWrite(rLed.pin, true);
//        delay(100);                              // change to 5100
//        digitalWrite(rLed.pin, false);
//      }
    }
  } else {
    int lineVal = getLineVal(rightSensor, leftSensor);
//    Serial.println(lineVal);
    //setMotors(lineFollower->getMotorSetting(lineVal));
    setMotors((Movement::MotorSetting){.speeds = {fSpeed, fSpeed}, .directions = {FORWARD, FORWARD}});
    //setMotors(getMovementFromStage(currentStage, lineVal));
//    setMotors(getMovementFromStage(LONG_TRAVERSE_0, lineVal));
  }

//  // led flashes if the motors are active
//  if (motorsActive) {
//    if (currentMillis - previousMillis >= oLedInterval) {
//      // save the last time you blinked the LED
//      previousMillis = currentMillis;
//  
//      // set the LED with the ledState of the variable:
//      digitalWrite(oLed.pin, !oLed.state);
//      oLed.state = !oLed.state;
//    }
//  }

  // scan for any commands in the BT or USB serial buffers
  if (Serial.available() > 0) {commandHandler(lineFollower, getSerialCommand());}
  if (SerialNina.available() > 0) {commandHandler(lineFollower, getBTSerialCommand());}

  //delay(5);
}
