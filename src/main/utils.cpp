#include "utils.h"
#include <WiFiNINA.h>
#include <SPI.h>
#include <Arduino.h>

Logger::Logger (unsigned long gap, Mode m) {
  timeGap = gap;
  mode = m;
}

// All of this code below  takes care of BT + serial communication and printing etc
void Logger::logln(int i) {
  unsigned long nowTime = millis();
  if (nowTime - lastLogTime >= timeGap) {
    if (mode == BOTH || mode == BT) {SerialNina.println(i);}
    if (mode == BOTH || mode == USB) {Serial.println(i);}
    lastLogTime = nowTime;
  }
}
void Logger::logln(String s) {
  unsigned long nowTime = millis();
  if (nowTime - lastLogTime >= timeGap) {
    if (mode == BOTH || mode == BT) {SerialNina.println(s);}
    if (mode == BOTH || mode == USB) {Serial.println(s);}
    lastLogTime = nowTime;
  }
}
void Logger::logln(float s) {
  unsigned long nowTime = millis();
  if (nowTime - lastLogTime >= timeGap) {
    if (mode == BOTH || mode == BT) {SerialNina.println(s);}
    if (mode == BOTH || mode == USB) {Serial.println(s);}
    lastLogTime = nowTime;
  }
}
void Logger::logln(double s) {
  unsigned long nowTime = millis();
  if (nowTime - lastLogTime >= timeGap) {
    if (mode == BOTH || mode == BT) {SerialNina.println(s);}
    if (mode == BOTH || mode == USB) {Serial.println(s);}
    lastLogTime = nowTime;
  }
}
void Logger::log(int i) {
  unsigned long nowTime = millis();
  if (nowTime - lastLogTime >= timeGap) {
    if (mode == BOTH || mode == BT) {SerialNina.print(i);}
    if (mode == BOTH || mode == USB) {Serial.print(i);}
    lastLogTime = nowTime;
  }
}
void Logger::log(String s) {
  unsigned long nowTime = millis();
  if (nowTime - lastLogTime >= timeGap) {
    if (mode == BOTH || mode == BT) {SerialNina.print(s);}
    if (mode == BOTH || mode == USB) {Serial.print(s);}
    lastLogTime = nowTime;
  }
}
void Logger::log(float i) {
  unsigned long nowTime = millis();
  if (nowTime - lastLogTime >= timeGap) {
    if (mode == BOTH || mode == BT) {SerialNina.print(i);}
    if (mode == BOTH || mode == USB) {Serial.print(i);}
    lastLogTime = nowTime;
  }
}
void Logger::log(double i) {
  unsigned long nowTime = millis();
  if (nowTime - lastLogTime >= timeGap) {
    if (mode == BOTH || mode == BT) {SerialNina.print(i);}
    if (mode == BOTH || mode == USB) {Serial.print(i);}
    lastLogTime = nowTime;
  }
}

int getLineVal(Sensor left, Sensor right) {
  int lineVal = 0;
  lineVal |= analogRead(right.pin) < 25; // change the read values based on line reading ==============================================================
  //delay(10);
  lineVal |= (analogRead(left.pin) < 25) << 1; // change the read values based on line reading ==============================================================
  return lineVal;
}

String getValsString(Sensor left, Sensor right) { // this function is to print out line follower sensor values
  int a  = analogRead(left.pin);
  //delay(10);
  int b  = analogRead(right.pin);
  //delay(10);
  return String(a) + " " + String(b);
}

// gets the colour of what is in front of the colour sensor
// won't be accurate unless distSensor is close enough
Color getColorVal(Sensor rLDR, Sensor bLDR) {
  int bVal = analogRead(bLDR.pin);
  int rVal = analogRead(rLDR.pin);
  if (bVal < rVal) { // Since rVal always seems to be reading more than bVal, change accordingly ==============================================================
    return BLUE;
  } else {
    return RED;
  }
}

void flashLed (unsigned long t, Led &led) {
  if (t - led.lastChanged >= led.interval) {
      // save the last time you blinked the LED
      led.lastChanged = t;
      // set the LED with the ledState of the variable:
      digitalWrite(led.pin, !led.state);
      led.state = led.state;
    }
}
