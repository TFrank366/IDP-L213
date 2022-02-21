// main.cpp
// the main program file for the arduino
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// number of each pin =========================
const int oLedPin =   2;
const int rLedPin =   3;
const int gLedPin =   4;
const int servoPin =  5;
const int os1Pin =    0;
const int os2Pin =    7;
const int os3Pin =    8;
// ============================================

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

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *m1 = AFMS.getMotor(1);
Adafruit_DCMotor *m2 = AFMS.getMotor(2);

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
  pinMode(os1Pin, INPUT);
  pinMode(os2Pin, INPUT);
  pinMode(os3Pin, INPUT);
  

  // initialise the motors
  AFMS.begin();
  m1->setSpeed(100);
  m2->setSpeed(100);
}

// supply motor

void startMotors(){
  // active led flashing if it's not already
  motorsActive = true;
  m1->run(FORWARD);
  m2->run(FORWARD);
  Serial.println("motors started");
}

void stopMotors() {
  // stop led flashing
  m1->run(RELEASE);
  m2->run(RELEASE);
  motorsActive = false;
  digitalWrite(oLed.pin, false);
  oLed.state = false;
  Serial.println("motors stopped");
}

unsigned long previousMillis = 0;
unsigned long interval = 500;
unsigned long elapsedTime = 0;


void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();
//  if (currentMillis > 10000 && motorsActive) {
//    stopMotors();
//  }


  int light = analogRead(os1.pin);
  //Serial.println(String(light));
  if (light < 100){
    Serial.println("white");
  } else {
    Serial.println("black");
  }
  
  
  
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
