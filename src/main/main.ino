// main.cpp
// the main program file for the arduino
#include <Servo.h>
#include <Adafruit_MotorShield.h>
#include <WiFiNINA.h>
#include "logger.h"
#include "movement.h"

// pin for each component =======================================================================
// digital pins
// port 2 is borked

const int oLedPin =         5; // orange LED (movement)
const int rLedPin =         3; // red LED (detection)
const int gLedPin =         4; // green LED (detection)
const int servo1Pin =       9;
const int servo2Pin =      10;

// analogue pins

// line sensor
const int rightsensorPin =        A2;  
const int leftsensorPin =         A0;  

// colour sensor
const int bLDRPin =        A3;  // Blue colour LDR voltage (goes down with more light)     
const int rLDRPin =        A4;  // Red colour LDR voltage (goes down with more light)

// distance sensor
const int distsensorPin =         A5;  // OPB704 Voltage (goes down with decreasing distance)       
          
// ==============================================================================================
const int fSpeed =        200; // motor speed for general movement
// ==============================================================================================

enum Dir {LEFT, RIGHT};

enum Color {BLUE, RED};

bool MOTORSREVERSED = false;
bool robotStopped = true;

struct Sensor {
  int pin;
  int value;
};

struct Led {
  int pin;
  bool state;
};

Logger l((unsigned long)0, USB);

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftwheel = AFMS.getMotor(1); 
Adafruit_DCMotor *rightwheel = AFMS.getMotor(2); 

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
// handles line following
Movement::FollowLine lineFollower(fSpeed, 50, (unsigned long)100);
// handles straight
Movement::Forward forward(fSpeed);
// handles stopped
Movement::Stop stopped;

void setup() {
  // setup serial link
  Serial.begin(9600);
  
  pinMode(NINA_RESETN, OUTPUT);         
  digitalWrite(NINA_RESETN, LOW);
  SerialNina.begin(115200);

  // machine startup acknowledgement
  Serial.println("hello world");

  //init led structs
  pinMode(oLedPin, OUTPUT);
  oLed = {.pin = oLedPin};
  pinMode(rLedPin, OUTPUT);
  rLed = {.pin = rLedPin};
  pinMode(gLedPin, OUTPUT);
  gLed = {.pin = gLedPin};

  // init the sensors
  pinMode(rightsensorPin, INPUT);
  rightsensor.pin = rightsensorPin;
  pinMode(leftsensorPin, INPUT);
  leftsensor.pin = leftsensorPin;
  pinMode(distsensorPin, INPUT);
  distsensor.pin = distsensorPin;
  pinMode(bLDRPin, INPUT);
  bLDR.pin = bLDRPin;
  pinMode(rLDRPin, INPUT);
  rLDR.pin = rLDRPin;

  // initialise the motors
  AFMS.begin();
  
}

// general function to apply a motorSetting struct onto the motors
void setMotors(Movement::MotorSetting mSetting) {
  leftwheel->setSpeed(mSetting.speeds[0]);
  rightwheel->setSpeed(mSetting.speeds[1]);
  
  // handle if the motors are reversed so forward -> backward
  if (!MOTORSREVERSED) {
    leftwheel->run(mSetting.directions[0]);
    rightwheel->run(mSetting.directions[1]);
  } else {
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
  
  if ((mSetting.speeds[0] > 0 || mSetting.speeds[1] > 0 ) && !motorsActive) {
    motorsActive = true;
  } else if (mSetting.speeds[0] == 0 && mSetting.speeds[1] == 0 && motorsActive) {
    motorsActive = false;
    robotStopped = true;
    digitalWrite(oLed.pin, false);
  }
}

int getLineVal(Sensor a, Sensor b) {
  int lineVal = 0;
  lineVal |= analogRead(a.pin) < 25; // both optoswitches apparently have different sensitivity to lighting
  delay(10);
  lineVal |= (analogRead(b.pin) < 800) << 1; // hence the difference in threshold value
  return lineVal;
}

// gets the colour of what is in front of the colour sensor
// won't be accurate unless depth sensor (distsensor) reads < 300
Color getColorVal(Sensor rLDR, Sensor bLDR) {
  int bVal = analogRead(bLDR.pin);
  int rVal = analogRead(rLDR.pin);
  if (bVal < rVal - 200){ // offset from red LDR needs to be corrected to give equal values when not picking up any red or blue
    return BLUE;
  } else {
    return RED;  
  }
}

String getValsString(Sensor s1, Sensor s2) {
  int a  = analogRead(s2.pin);
  delay(10);
  int c  = analogRead(s1.pin);
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
  } else if (command.substring(0, 2) == "lf") {
    // this will likely only happen if the message has been sent double
    l.logln("line follower change");
    if (command.length() > 16) {return;}
    // line following parameter change on the fly
    String params = command.substring(3);
    // contains all the parameters separated by spaces

    String forwardSpeedStr = params.substring(0, params.indexOf(" "));
    l.logln(forwardSpeedStr);
  
    String turnAmountStr = params.substring(params.indexOf(" ") + 1, params.lastIndexOf(" "));
    l.logln(turnAmountStr);

    // trim out the first number and space
    String turnDurationStr = params.substring(params.lastIndexOf(" ") + 1);
    l.logln(turnDurationStr);
    
    // modify the line following algorithm with the received parameters
    lineFollowerPtr->setParams(forwardSpeedStr.toInt(), turnAmountStr.toInt(), (unsigned long)turnDurationStr.toInt());
//    lineFollower.fSpeed = forwardSpeedStr.toInt();
//    lineFollower.turnAmount = turnAmountStr.toInt();
//    lineFollower.turnDuration = (unsigned long)turnDurationStr.toInt();
    l.logln("lf change done");
  }
}


unsigned long previousMillis = 0;
unsigned long oLedInterval = 500;

void loop() {
  //put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();

  //servo.write(min(180, int(currentMillis/100)));

  // the main movement code 
  if (robotStopped) {
    setMotors(stopped.getMotorSetting());
    //l.logln(analogRead(distsensor.pin));
    if (analogRead(distsensor.pin) < 800) {
      delay(10);
      l.logln(String(analogRead(rLDR.pin)) + " " + String(analogRead(bLDR.pin)));
      delay(10);
      Color blockCol = getColorVal(rLDR, bLDR);
      if (blockCol == BLUE) {
        l.logln("blue");
        digitalWrite(gLed.pin, true);
        delay(100); // change to 5100 for actual!
        digitalWrite(gLed.pin, false);
      } else {
        l.logln("red");
        digitalWrite(rLed.pin, true);
        delay(100); // change to 5100 for actual!
        digitalWrite(rLed.pin, false);
      }
    }
  } else {
    //int lineVal = getLineVal(rightsensor, os2, leftsensor);
    //l.logln(String(analogRead(leftsensor.pin)) + " " + String(analogRead(os2.pin)) + " " + String(analogRead(rightsensor.pin)));
    //l.logln(getValsString(rightsensor, leftsensor));
//    delay(10);
    int lineVal = getLineVal(rightsensor, leftsensor);
    //l.logln(lineVal);
//    if (lightToBit(analogRead(rightsensor.pin)) == 1 && lightToBit(analogRead(leftsensor.pin)) == 1) {
//      setMotors(stopped.getMotorSetting());
//    } else {
//      setMotors(forward.getMotorSetting());
//      //setMotors(lineFollower.getMotorSetting(lineVal));
//    }  
    setMotors(lineFollower.getMotorSetting(lineVal));
    //setMotors(stopped.getMotorSetting());
  }

  // led flashes if the motors are active
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
  if (Serial.available() > 0) {commandHandler(&lineFollower, getSerialCommand());}
  if (SerialNina.available() > 0) {commandHandler(&lineFollower, getBTSerialCommand());}
}
