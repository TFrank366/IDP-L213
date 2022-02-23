// main.cpp
// the main program file for the arduino
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// number of each pin =========================
// port 2 is borked
const int oLedPin =   5;
const int rLedPin =   3;
const int gLedPin =   4;
const int servo1Pin =  9;
const int servo2Pin = 10;
const int os1Pin =    0;  // analog pin #
const int os2Pin =    1;  // analog pin #
const int os3Pin =    2;  // analog pin #
// ============================================
const int fSpeed =  100;  // motor speed for general movement
// ============================================

enum Dir {LEFT, RIGHT};

struct Motor {
  int pin;
  int currSpeed;
};

struct Sensor {
  int pin;
  int value;
};

struct Led {
  int pin;
  bool state;
};

struct TurnCommand {
  Dir dir;
  bool active;
  int amount;             // the angular speed of the turn in some units
  unsigned long start;
  unsigned long duration;
};

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *m1 = AFMS.getMotor(1);
Adafruit_DCMotor *m2 = AFMS.getMotor(2);

Servo servo;

// create led structs
Led oLed;
Led rLed;
Led gLed;

//create sensor structs 
//optoswitches
Sensor os1;
Sensor os2;
Sensor os3;

bool motorsActive = false;
int currSpeed = 0; // keep a track of the current speed

void setup() {
  // setup serial link
  Serial.begin(9600);

  //init led structs
  pinMode(oLedPin, OUTPUT);
  oLed = {.pin = oLedPin};
  pinMode(rLedPin, OUTPUT);
  rLed = {.pin = rLedPin};
  pinMode(gLedPin, OUTPUT);
  gLed = {.pin = gLedPin};

  // init the sensors
  os1.pin = os1Pin;
  os2.pin = os2Pin;
  os3.pin = os3Pin;
  pinMode(os1Pin, INPUT);
  pinMode(os2Pin, INPUT);
  pinMode(os3Pin, INPUT);

  

  // initialise the motors
  AFMS.begin();
  setMotorsStraight(fSpeed);

  // init the servo
  servo.attach(servo1Pin);

  //start moving forward
  startMotors();
}


void setMotorsStraight(int mSpeed) {
  currSpeed = mSpeed;
  m1->setSpeed(mSpeed);
  m2->setSpeed(mSpeed);
  //Serial.println("motors same");
}

void startMotors(){
  // active led flashing if it's not already
  motorsActive = true;
  m1->run(FORWARD);
  m2->run(FORWARD);
  //Serial.println("motors started");
}

void stopMotors() {
  // stop led flashing
  m1->run(RELEASE);
  m2->run(RELEASE);
  motorsActive = false;
  digitalWrite(oLed.pin, false);
  oLed.state = false;
  //Serial.println("motors stopped");
}

// 0 is black 1 is white
int lightToCol(int light) {
  if (light < 150){
    return 1;
  } else {
    return 0;
  }
}

//                           4  2  1
// returns an int with bits (s3 s2 s1)
// line on left => 100 => 4
int getColsVal(Sensor s1, Sensor s2, Sensor s3) {
  int colsVal = 0;
  colsVal |= lightToCol(analogRead(s1.pin));
  colsVal |= lightToCol(analogRead(s2.pin)) << 1;
  colsVal |= lightToCol(analogRead(s3.pin)) << 2;
  return colsVal;
}

TurnCommand getTurnCommand(int colsVal){
    unsigned long nowMillis = millis();
    TurnCommand turn;
    // Serial.println(colsVal);
    switch (colsVal) {
      // line only on right sensor
      // have to turn right
      case 1:
      case 3:  
        Serial.println("right");
        turn = {RIGHT, true, 50, nowMillis, 100};
        break;
       // line only on left sensor
       // have to turn left
       case 4:
       case 6:
        Serial.println("left");
        turn = {LEFT, true, 50, nowMillis, 100};
        break;
       default:
       turn.active = false;
        break;
    }
    return turn;
}

                
// robot is setup m1---m2
//                 \ ^ /
//                  \o/
void startTurn(TurnCommand turn) {
  switch(turn.dir) {
    case LEFT:
      m2->setSpeed(int(currSpeed + turn.amount/2));
      m1->setSpeed(max(int(currSpeed - turn.amount/2), 0)); // set to 0 if its too low TEMP
      break;
    case RIGHT:
      m2->setSpeed(int(currSpeed - turn.amount/2));
      m1->setSpeed(max(int(currSpeed + turn.amount/2), 0)); // set to 0 if its too low TEMP
      break;
  }
}

void printVals(Sensor s1, Sensor s2, Sensor s3) {
  Serial.print(lightToCol(analogRead(s1.pin)));
  Serial.print(lightToCol(analogRead(s2.pin)));
  Serial.println(lightToCol(analogRead(s3.pin)));
}

unsigned long previousMillis = 0;
unsigned long interval = 500;
unsigned long elapsedTime = 0;


TurnCommand turn;

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();

  servo.write(min(180, int(currentMillis/100)));

  
  // if the timer of the current turn command has elapsed, deacivate it
  if (currentMillis > turn.start + turn.duration && turn.active) {
    // set the active flag to off
    turn.active = false;
    // stop turning and move straight
    setMotorsStraight(fSpeed);
    //Serial.println("stopped turn");
  }
  
  // if the robot is currently turning, dont worry about this
  if (!turn.active){
    int colsVal = getColsVal(os1, os2, os3);
    //Serial.println(colsVal);
    //Serial.println(colsVal);//std::bitset<3>(colsVal))
    turn = getTurnCommand(colsVal);
    if (turn.active) {
      startTurn(turn);
    }
    
    printVals(os1, os2, os3);
  }
  


  // led flashes if the motors are active
  if (motorsActive) {
    if (currentMillis - previousMillis >= interval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;
  
      // set the LED with the ledState of the variable:
      digitalWrite(oLed.pin, !oLed.state);
      oLed.state = !oLed.state;
    }
  }
}
