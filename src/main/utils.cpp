#include "utils.h"
#include <WiFiNINA.h>
#include <SPI.h>
#include <Arduino.h>

// TODO: impelement per channel time gap

Logger::Logger (unsigned long gap, Mode m) {
  timeGap = gap;
  mode = m;
}
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
