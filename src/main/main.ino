// main.cpp
// the main program file for the arduino
#include <Servo.h>;
#include <Wire.h>;
#include <Adafruit_MotorShield.h>;

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
Adafruit_DCMotor *m1 = AFMS.getMotor(1);
Adafruit_DCMotor *m2 = AFMS.getMotor(2);

// creat output control structs
Led oLed;

bool motorsActive = false;

void setup() {
  // set the correct pins to output
  //init led structs
  pinMode(oLedPin, OUTPUT);
  oLed = {.pin = oLedPin};

  AFMS.begin();
  m1->setSpeed(200);
  m1->run(FORWARD);
  m2->setSpeed(200);
  m2->run(FORWARD);
//  // create servo motor
//  Servo grabberServo
//  grabberServo.attach(servoPin)
  
}

// supply motor

void activateMotors(){
  // active led flashing if it's not already
  motorsActive = true;
  m1->run(FORWARD);
  m2->run(FORWARD);
  std::cout<<"motors started";
}

// set the speed
void setMotorSpeed(Motor m, int s) {
  
}
void stopMotors() {
  // stop led flashing
  m1->run(RELEASE);
  m2->run(RELEASE);
  motorsActive = false;
  digitalWrite(oLed.pin, false);
  oLed.state = false;
}

unsigned long previousMillis = 0;
unsigned long interval = 500;
unsigned long elapsedTime = 0;


void activateMotors();

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();
  if (currentMillis > 5000) {
    void stopMotors();
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
