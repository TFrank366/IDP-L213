// main.cpp
// the main program file for the arduino
const int m1Pin = 0;
const int m2Pin = 1;
const int oLedPin = 2;
const int rLedPin = 3;
const int gLedPin = 4;

struct Motor {
  int pin;
  int currSpeed = 0;
};

struct Led {
  int pin;
  bool state = false;
};


void setup() {
  // set the correct pins to output
  pinMode(m1Pin, OUTPUT);
  pinMode(m2Pin, OUTPUT);
  // create motor structs
  Motor m1{m1Pin};
  Motor m2{m2Pin};
  //create led structs
  Led oLed{oLedPin};
}

// supply motor
void setMotorSpeed(Motor m, int s) {

}
void activeMotors(){
  // active led flashing if it's not already
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
