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
const int oLedPin =         5;
const int rLedPin =         3;
const int gLedPin =         4;
const int servo1Pin =       9;
const int servo2Pin =      10;
// analogue pins
// line sensor
const int os1Pin =         A2;  // analogue pin #
const int os2Pin =         A1;  // analogue pin #
const int os3Pin =         A0;  // analogue pin #
// colour sensor
const int bLDRPin =        A3;  // Blue colour LDR voltage (goes down with more light)        TBD
const int rLDRPin =        A4;  // Red colour LDR voltage                                     TBD
const int os4Pin =         A5;  // OPB704 Voltage (goes down with decreasing distance)        TBD
const int wLedPin =         6;  // pin that the LED is attached to              TBD
// ==============================================================================================
const int fSpeed =        200; // motor speed for general movement
// ==============================================================================================

enum Dir {LEFT, RIGHT};

enum Color {BLUE, RED};

bool MOTORSREVERSED = false;
bool robotStopped = true;

//struct Motor {
//  int pin;
//  int currSpeed;
//};

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
Adafruit_DCMotor *m1 = AFMS.getMotor(1);
Adafruit_DCMotor *m2 = AFMS.getMotor(2);

Servo servo;

// create led structs
Led oLed;
Led rLed;
Led gLed;
Led wLed;

//create sensor structs 
//optoswitches
Sensor os1;
Sensor os2;
Sensor os3;
Sensor os4;
//LDRs
Sensor rLDR;
Sensor bLDR;


// global flag to keep track of if the motors are running to know when to flash oLed
bool motorsActive = false;
//int currSpeed = 0; // keep a track of the current speed
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

  //SerialNina.write(SerialNina.read());
  Serial.println("hello world");

  //init led structs
  pinMode(oLedPin, OUTPUT);
  oLed = {.pin = oLedPin};
  pinMode(rLedPin, OUTPUT);
  rLed = {.pin = rLedPin};
  pinMode(gLedPin, OUTPUT);
  gLed = {.pin = gLedPin};
  pinMode(wLedPin, OUTPUT);
  wLed = {.pin = wLedPin};

  // init the sensors
  pinMode(os1Pin, INPUT);
  os1.pin = os1Pin;
  pinMode(os2Pin, INPUT);
  os2.pin = os2Pin;
  pinMode(os3Pin, INPUT);
  os3.pin = os3Pin;
  pinMode(os4Pin, INPUT);
  os4.pin = os4Pin;
  pinMode(bLDRPin, INPUT);
  bLDR.pin = bLDRPin;
  pinMode(rLDRPin, INPUT);
  rLDR.pin = rLDRPin;

  // initialise the motors
  AFMS.begin();
  
}


// general function to apply a motorSetting struct onto the motors
void setMotors(Movement::MotorSetting mSetting) {
  //l.logln("motors set");
  m1->setSpeed(mSetting.speeds[0]);
  m2->setSpeed(mSetting.speeds[1]);
  
  // handle if the motors are reversed so forward -> backward
  if (!MOTORSREVERSED) {
    m1->run(mSetting.directions[0]);
    m2->run(mSetting.directions[1]);
  } else {
    if (mSetting.directions[0] == FORWARD) {
      m1->run(BACKWARD);
    } else {
      m1->run(FORWARD);
    }
    if (mSetting.directions[1] == FORWARD) {
      m2->run(BACKWARD);
    } else {
      m2->run(FORWARD);
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

// 0 is black 1 is white
int lightToBit(int light) {
  if (light < 75){
    return 1;
  } else {
    return 0;
  }
}

//                           4  2  1
// returns an int with bits (s3 s2 s1)
// line on left => 100 => 4
int getLineVal(Sensor s1, Sensor s2, Sensor s3) {
  int lineVal = 0;
  lineVal |= analogRead(s1.pin) < 25;
  delay(10);
  lineVal |= lightToBit(analogRead(s2.pin)) << 1;
  delay(10);
  lineVal |= (analogRead(s3.pin) < 800) << 2;
  return lineVal;
}

// gets the colour of what is in front of the colour sensor
// won't be accurate unless depth sensor (os4) reads < 300
Color getColorVal(Sensor rLDR, Sensor bLDR) {
  int bVal = analogRead(bLDR.pin);
  int rVal = analogRead(rLDR.pin);
  if (bVal < rVal - 200){
    return BLUE;
  } else {
    return RED;  
  }
}

String getValsString(Sensor s1, Sensor s2, Sensor s3) {
  int a  = analogRead(s3.pin);
  delay(10);
  int b  = analogRead(s2.pin);
  delay(10);
  int c  = analogRead(s1.pin);
  delay(10);
  return String(a) + " " + String(b) + " " + String(c);
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
    //l.logln(analogRead(os4.pin));
    if (analogRead(os4.pin) < 800) {
      delay(10);
      l.logln(String(analogRead(rLDR.pin)) + " " + String(analogRead(bLDR.pin)));
      delay(10);
      Color blockCol = getColorVal(rLDR, bLDR);
      if (blockCol == BLUE) {
        l.logln("blue");
        digitalWrite(gLed.pin, true);
        delay(100);
        digitalWrite(gLed.pin, false);
      } else {
        l.logln("red");
        digitalWrite(rLed.pin, true);
        delay(100);
        digitalWrite(rLed.pin, false);
      }
    }
  } else {
    //int lineVal = getLineVal(os1, os2, os3);
    //l.logln(String(analogRead(os3.pin)) + " " + String(analogRead(os2.pin)) + " " + String(analogRead(os1.pin)));
    //l.logln(getValsString(os1, os2, os3));
//    delay(10);
    int lineVal = getLineVal(os1, os2, os3);
    //l.logln(lineVal);
//    if (lightToBit(analogRead(os1.pin)) == 1 && lightToBit(analogRead(os3.pin)) == 1) {
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
