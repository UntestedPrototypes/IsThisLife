#pragma once
#include <Arduino.h>

// ================= EXTERN GLOBALS =================
extern uint8_t ESP32_ID;
extern float x;
extern float y;

// ================= PUBLIC API =================

// WiFi lifecycle
void wiFiInit();
void wiFiWatchdog();

// UDP
void parseUdp();
void sendToLaptop(const String& msg);

// I2C
bool checkI2CDevicesAndReport(bool report = true);
