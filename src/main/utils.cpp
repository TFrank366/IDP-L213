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

RisingEdgeDetector::RisingEdgeDetector() {
  historyLength = sizeof(history)/sizeof(bool);
}

void RisingEdgeDetector::addNew(bool value) {
}

// newer values are pushed in the front of the array (lower index)
bool RisingEdgeDetector::checkForEdge() {
  int highCount = 0;
  int lowCount = 0;
  int edgeIndex = -1;
  for (int i = 0; i < historyLength; i++) {
    if (history[i] == true && edgeIndex == -1) {
      // value is true before edge - good
      highCount++;
    } else {
      // value is false
      if (i > historyLength/4 && edgeIndex == -1) {
        edgeIndex = i;
        lowCount++;
      } else if (i > edgeIndex) {
        lowCount++;
      }
    }
  }
  //check if the proportions are enough to consitiute a true rising edge
  if ((float)highCount/edgeIndex > 0.75 && (float)lowCount/(historyLength - edgeIndex)) {
    return true;
  } else {
    return false;
  }
}

void RisingEdgeDetector::reset() {
  for (int i = 0; i < historyLength; i++) {
    history[i] = true;
  }
}
