#include "communication.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>



// ================= WIFI / UDP =================
static const char* WIFI_SSID = "isthislife?";
static const char* WIFI_PASS = "$Admin1234";
static IPAddress broadcastIP(255, 255, 255, 255);
static const uint16_t UDP_PORT = 4210;

#define LAPTOP_ID 0

// ================= PRIVATE STATE =================
static WiFiUDP udp;
static char rxBuf[128];
static uint32_t lastWifiOkMs = 0;

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

    // Parse starting from index 1 (skips the ID byte)
    if (sscanf(&rxBuf[1], "%f,%f", &x, &y) == 2) {
        // Only send confirmation if the parse was successful
        // sendToLaptop("Received command: x=" + String(x) + ", y=" + String(y));
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
