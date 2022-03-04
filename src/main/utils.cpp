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
  for (int i = historyLength - 1; i > 0; i--) {
    history[i] = history[i - 1];
  }
  history[0] = value;
}

// newer values are pushed in the front of the array (lower index)
bool RisingEdgeDetector::checkForEdge() {
  int highCount = 0;
  int lowCount = 0;
  int edgeIndex = -1;
  for (int i = 0; i < historyLength; i++) {
    if (history[i] == true && edgeIndex == -1) {
//      Serial.print("high ");
//      Serial.println(i);
      // value is true before edge - good
      highCount++;
    } else if (history[i] == false){
      // value is false
      // need at least a quarter of the history high to make sure
      if (i > historyLength/4 && edgeIndex == -1) {
        edgeIndex = i;
//        Serial.print("low ");
//        Serial.println(i);
        lowCount++;
      } else if (i > edgeIndex) {
//        Serial.print("low ");
//        Serial.println(i);
        lowCount++;
      }
    }
  }
  //check if the proportions are enough to consitiute a true rising edge
//  Serial.println((float)highCount/edgeIndex);
//  Serial.println((float)lowCount/(historyLength - edgeIndex));
  if ((float)highCount/edgeIndex > 0.5 && (float)lowCount/(historyLength - edgeIndex) > 0.5) {
    return true;
  } else {
    return false;
  }
}

void RisingEdgeDetector::set(bool value) {
  for (int i = 0; i < historyLength; i++) {
    history[i] = value;
  }
}
