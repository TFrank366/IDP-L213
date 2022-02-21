#include <Adafruit_CircuitPlayground.h>
#include <Adafruit_Circuit_Playground.h>

// main.cpp
// the main program file for the arduino
#include <Servo.h>;
#include <Wire.h>;
#include <Adafruit_MotorShield.h>;
#include "utility/Adafruit_MS_PWMServoDriver.h"

int m1Pin = 0;
int m2Pin = 1;
int oLedPin = 2;
int rLedPin = 3;
int gLedPin = 4;
int servoPin = 5;

struct Motor {
  int pin;
  int currSpeed;
};

struct Led {
  int pin;
  bool state;
};

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);

// creat output control structs
Motor m1;
Motor m2;
Led oLed;

void setup() {
  // set the correct pins to output
  pinMode(m1Pin, OUTPUT);
  pinMode(m2Pin, OUTPUT);
  // init motor structs
  m1 = {m1Pin, 0};
  m2 = {m2Pin, 0};
  //init led structs
  pinMode(oLedPin, OUTPUT);
  oLed = {.pin = oLedPin};

  AFMS.begin()
  myMotor->setSpeed(50);
  myMotor->run(FORWARD);
  delay(2000);
  myMotor->setSpeed(0);
//  // create servo motor
//  Servo grabberServo
//  grabberServo.attach(servoPin)
  
}

// supply motor

void activeMotors(){
  // active led flashing if it's not already
}
// set the speed
void setMotorSpeed(Motor m, int s) {
}
void stopMotors() {
  // stop led flashing
}

unsigned long previousMillis = 0;
unsigned long interval = 500;

void loop() {
  // put your main code here, to run repeatedly:
  
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // set the LED with the ledState of the variable:
    digitalWrite(oLed.pin, !oLed.state);
    oLed.state = !oLed.state;
  }
}
