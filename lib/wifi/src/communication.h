#pragma once
#include <Arduino.h>

// ================= EXTERN GLOBALS =================
extern uint8_t ESP32_ID;
extern float x;
extern float y;
extern bool button_pressed;
extern bool emergency_stop;
extern uint32_t lastValidUdpTime;

// ================= PUBLIC API =================

// WiFi lifecycle
void wiFiInit();
void wiFiWatchdog();

// UDP
void parseUdp();
void sendToLaptop(const String& msg);
bool checkConnectionTimeout(uint32_t timeoutMs);

// I2C
bool checkI2CDevicesAndReport(bool report = true);
