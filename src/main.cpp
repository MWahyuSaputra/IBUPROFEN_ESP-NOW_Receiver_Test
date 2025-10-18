#define DEBUG
#ifdef DEBUG
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
  #define DEBUG_BEGIN(baud) Serial.begin(baud)
#else
  #define DEBUG_PRINT(...)    // Do nothing
  #define DEBUG_PRINTLN(...)  // Do nothing
  #define DEBUG_BEGIN(baud)   // Do nothing
#endif

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "ESPNOW_Functions.h"

void setup() {
  DEBUG_BEGIN(115200);
  WiFi.mode(WIFI_STA);
  setupGripperControl();
}

void loop()
{  
  startComms();
}