// main.cpp
// the main program file for the arduino
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MotorShield.h>
//#include <Arduino_LSM6DS3.h>
#include <WiFiNINA.h>
#include "utils.h"
#include "movement.h"
//#include "robot.h"

// pin for each component =======================================================================
// digital pins
// port 2 is borked

const int oLedPin =         4; // orange LED (movement)
const int rLedPin =         3; // red LED (detection)
const int gLedPin =         5; // green LED (detection)
const int servo1Pin =      10;
const int servo2Pin =       9;

// analogue pins

// line sensor
const int rightsensorPin = A4;  
const int leftsensorPin =  A5; 
 
// colour sensor
const int bLDRPin =        A2;  // Blue colour LDR voltage (goes down with more light)     
const int rLDRPin =        A3;  // Red colour LDR voltage (goes down with more light)

// distance sensor
const int distsensorPin =  A0;  // OPB704 Voltage (goes down with decreasing distance)       
          
// ==============================================================================================
const int fSpeed =        150; // motor speed for general movement
// ==============================================================================================

enum Dir {LEFT, RIGHT};

//enum Color {BLUE, RED};

// if the directions of the motors need to be reversed
bool MOTORSREVERSED = false;
bool robotStopped = true;
//


Logger l((unsigned long)200, BOTH);

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftwheel = AFMS.getMotor(4); 
Adafruit_DCMotor *rightwheel = AFMS.getMotor(3); 

Servo servo;

// create led structs
Led oLed;
Led rLed;
Led gLed;

//create sensor structs 

//optoswitches
Sensor rightsensor;
Sensor leftsensor;
Sensor distsensor;

//LDRs
Sensor rLDR;
Sensor bLDR;

// global flag to keep track of if the motors are running to know when to flash oLed
bool motorsActive = false;

// create object for handling the different movement regimes
// handles line following         speed  angular v     duration
Movement::FollowLine lineFollower(80, 10, (unsigned long)100);
// handles straight forward movement
Movement::Straight forward(fSpeed);
// handles stopped
Movement::Stop stopped;

//Robot::Vehicle* robot(fSpeed, 90, 200, (unsigned long)100);

//Robot::programStage program[3];
//int currentStageNum = 0;


void setup() {
  // setup serial link
  Serial.begin(9600);

  // program the robot
//  program[0] = (Robot::programStage){.stageName=Robot::MOVE_TO_BLOCK, .next = 1};
//  program[1] = (Robot::programStage){.stageName=Robot::SENSE_BLOCK_COLOR, .next = 2};
//  program[2] = (Robot::programStage){.stageName=Robot::LONG_TRAVERSE_0, .next = 0};
  
//  Serial.println(robot->program[0].stageName);
//  Serial.println(Robot::MOVE_TO_BLOCK);
  //IMU.begin();
  
  pinMode(NINA_RESETN, OUTPUT);         
  digitalWrite(NINA_RESETN, LOW);
  SerialNina.begin(115200);

  // machine startup acknowledgement
  Serial.println("hello world");

  //init led structs
  pinMode(oLedPin, OUTPUT);
  pinMode(rLedPin, OUTPUT);
  pinMode(gLedPin, OUTPUT);
  
  oLed = {.pin = oLedPin};
  rLed = {.pin = rLedPin};
  gLed = {.pin = gLedPin};
  

  // init the sensors
  pinMode(rightsensorPin, INPUT);
  pinMode(leftsensorPin, INPUT);
  pinMode(distsensorPin, INPUT);
  pinMode(bLDRPin, INPUT);
  pinMode(rLDRPin, INPUT);
  

  rightsensor.pin = rightsensorPin;
  leftsensor.pin = leftsensorPin;
  distsensor.pin = distsensorPin;
  bLDR.pin = bLDRPin;
  rLDR.pin = rLDRPin;

  

  // initialise the motors
  AFMS.begin();

  //servo.attach(servo2Pin);
  
  // 162 is fully closed
  // 105 is open
  
}

// general function to apply a motorSetting struct onto the motors
void setMotors(Movement::MotorSetting mSetting) {
  Serial.println("set motors");
  Serial.println(mSetting.speeds[0]);
  Serial.println(mSetting.speeds[1]);
  leftwheel->setSpeed(mSetting.speeds[0]);
  rightwheel->setSpeed(mSetting.speeds[1]);
  Serial.println("set motors middle");
  
  // handle if the motors are reversed so forward -> backward
  if (!MOTORSREVERSED) {
    leftwheel->run(mSetting.directions[0]);
    rightwheel->run(mSetting.directions[1]);
    if (mSetting.directions[0] == FORWARD) {
      leftwheel->run(BACKWARD);
    } else {
      leftwheel->run(FORWARD);
    }
    if (mSetting.directions[1] == FORWARD) {
      rightwheel->run(BACKWARD);
    } else {
      rightwheel->run(FORWARD);
    }
  }
  Serial.println("end of set motors");
  
  if ((mSetting.speeds[0] > 0 || mSetting.speeds[1] > 0 ) && !motorsActive) {
    motorsActive = true;
  } else if (mSetting.speeds[0] == 0 && mSetting.speeds[1] == 0 && motorsActive) {
    motorsActive = false;
    robotStopped = true;
    digitalWrite(oLed.pin, false);
  }
}

// a is right b is left
int getLineVal(Sensor a, Sensor b) {
  int lineVal = 0;
  lineVal |= analogRead(a.pin) < 850; // both optoswitches apparently have different sensitivity to lighting
  delay(10);
  lineVal |= (analogRead(b.pin) < 850) << 1; // hence the difference in threshold value
  return lineVal;
}

// gets the colour of what is in front of the colour sensor
// won't be accurate unless depth sensor (distsensor) reads < 300
Color getColorVal(Sensor rLDR, Sensor bLDR) {
  int bVal = analogRead(bLDR.pin);
  int rVal = analogRead(rLDR.pin);
  //Serial.println("red: " + String(rVal) + " blue: " + String(bVal));
  if (bVal < rVal + 200){ // offset from red LDR needs to be corrected to give equal values when not picking up any red or blue
    return BLUE;
  } else {
    return RED;  
  }
}

// s1 is right s2 is left
String getValsString(Sensor s1, Sensor s2) {
  int a  = analogRead(s2.pin) < 850;
  delay(10);
  int c  = analogRead(s1.pin) < 850;
  delay(10);
  return String(a) + " " + String(c);
}

// serial command receiver
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

void commandHandler(String command) {
  // print the command string that was received
  Serial.println(command);
  l.logln("command received");
  
  if (command == "stop" || command == "stopstop") {
    Serial.println("stopping");
    robotStopped = true;
  } else if (command == "go" || command == "gogo") {
    l.logln("starting");
    robotStopped = false;
  } 
}


unsigned long previousMillis = 0;
unsigned long oLedInterval = 500;

bool servoOn = true;

void loop() {
  Serial.println("millis");
  unsigned long currentMillis = millis();
//  servo.write(max(min(currentMillis/80+90, 162), 105));
//  if (currentMillis > 7000 && servoOn) {
//    l.logln("servo detached");
//    servo.detach();
//    servoOn = false;
//  }

  //l.logln(getValsString(rightsensor, leftsensor));
  if (!robotStopped) {
   // check if the conditions have been met to advance the stage to the next
//   if (robot->checkForAdvance(program[currentStageNum].stageName)) {
//      // set the stage index to the next one
//      currentStageNum = program[currentStageNum].next;
//      // call the robot's method to advance 
//      robot->advanceStage();
//    }
    // set the motors correctly for the current stage
    //setMotors(robot->getMotorSetting(program[currentStageNum].stageName, getLineVal( rightsensor, leftsensor)));
    Serial.println("main loop");
    //getLineVal(rightsensor, leftsensor);
    //setMotors(lineFollower.getMotorSetting(getLineVal(rightsensor, leftsensor)));
    setMotors(forward.getMotorSetting());
    //setMotors(robot->getMotorSetting(Robot::LONG_TRAVERSE_0, getLineVal(rightsensor, leftsensor)));
    //setMotors((Movement::MotorSetting){{100, 0}, {FORWARD, FORWARD}});
    // perform any applicable functions for the current stage
    //robot->performFunction(program[currentStageNum].stageName, analogRead(distsensor.pin), getColorVal(rLDR, bLDR), gLed, rLed);
    //robot->performFunction(Robot::SENSE_BLOCK_COLOR, analogRead(distsensor.pin), getColorVal(rLDR, bLDR), gLed, rLed);
  } else {
    //setMotors(stopped.getMotorSetting()); 
  }

  // led flashes if the motors are active
  if (motorsActive) {
    //Serial.println("led flashing");
    if (currentMillis - previousMillis >= oLedInterval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;
  
      // set the LED with the ledState of the variable:
      digitalWrite(oLed.pin, !oLed.state);
      oLed.state = !oLed.state;
    }
  }

  // scan for any commands in the BT or USB serial buffers
  if (Serial.available() > 0) {commandHandler(getSerialCommand());}
  if (SerialNina.available() > 0) {commandHandler(getBTSerialCommand());}
  Serial.println("end of loop");
}
