#include "communication.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>



// ================= WIFI / UDP =================
static const char* WIFI_SSID = "Nothing(2)";
static const char* WIFI_PASS = "B13pB00pY33t";
static IPAddress broadcastIP(255, 255, 255, 255);
static const uint16_t UDP_PORT = 4210;

#define LAPTOP_ID 0

// ================= PRIVATE STATE =================
static WiFiUDP udp;
static char rxBuf[128];
static uint32_t lastWifiOkMs = 0;

// ================= PUBLIC STATE =================
uint32_t lastValidUdpTime = 0;

// ================= WIFI INIT =================
void wiFiInit() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.setSleep(false);

  udp.begin(UDP_PORT);
  lastWifiOkMs = millis();
}

// ================= UDP OUT =================
void sendToLaptop(const String& msg) {
  udp.beginPacket(broadcastIP, UDP_PORT);
  udp.print(LAPTOP_ID);
  udp.print("ESP32" + String(ESP32_ID));
  udp.print(msg);
  udp.endPacket();
}

// ================= I2C CHECK =================
bool checkI2CDevicesAndReport(bool report) {
  struct Dev { uint8_t addr; const char* name; };
  Dev devices[] = {
    {0x28, "IMU_R"},
    {0x29, "IMU_L"},
    {0x18, "MCP9808"},
    {0x40, "INA219"},
  };

  bool all_ok = true;
  String msg = "I2C";

  for (auto& d : devices) {
    Wire.beginTransmission(d.addr);
    if (Wire.endTransmission() != 0) {
      all_ok = false;
      msg += ",FAIL_";
      msg += d.name;
    }
  }

  if (report) {
    sendToLaptop(all_ok ? "I2C,OK" : msg);
  }

  return all_ok;
}

// ================= UDP INPUT =================
void parseUdp() {
    int sz = udp.parsePacket();
    if (sz <= 0) return;

    int len = udp.read(rxBuf, sizeof(rxBuf) - 1);
    if (len <= 0) return;

    rxBuf[len] = 0; // Null-terminate for safety


    // --- HYBRID ID CHECK ---
    // myIdChar turns number 1 into character '1' (decimal 49)
    char myIdChar = (char)('0' + ESP32_ID); 
    
    // Check if the packet starts with the numerical 1 OR the text '1'
    if (rxBuf[0] != ESP32_ID && rxBuf[0] != myIdChar) {
        return;
    }

    // Update last valid UDP timestamp
    lastValidUdpTime = millis();

    // Extract command byte (rxBuf[1])
    char cmd = rxBuf[1];

    // Check for emergency stop (format: "1E")
    if (cmd == 'E') {
        emergency_stop = true;
        x = 0.0f;
        y = 0.0f;
        sendToLaptop("EMERGENCY STOP ACTIVATED!");
        return;
    }

    // Check for emergency stop reset (format: "1R")
    if (cmd == 'R') {
        emergency_stop = false;
        sendToLaptop("Emergency stop cleared - resuming normal operation");
        return;
    }

    // Check for button press (format: "1B")
    if (cmd == 'B') {
        button_pressed = true;
        sendToLaptop("Button acknowledged");
        return;
    }

    // Check for control packet (format: "1X+0.123,-0.456")
    if (cmd == 'X' && emergency_stop == false) {
        if (sscanf(&rxBuf[2], "%f,%f", &x, &y) == 2) {
            // Successfully parsed control values
        }
        return;
    }
}

// ================= WIFI WATCHDOG =================
void wiFiWatchdog() {
  if (WiFi.status() == WL_CONNECTED) {
    lastWifiOkMs = millis();
  } else if (millis() - lastWifiOkMs > 3000) {
    ESP.restart();
  }
}

// ================= CONNECTION TIMEOUT CHECK =================
bool checkConnectionTimeout(uint32_t timeoutMs) {
  // Returns true if no valid UDP packet received within timeoutMs
  if (lastValidUdpTime == 0) {
    return false; // No packets received yet, not a timeout
  }
  return (millis() - lastValidUdpTime) > timeoutMs;
}
