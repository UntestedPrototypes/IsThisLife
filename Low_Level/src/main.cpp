#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ===== CONFIG =====
#define ESP32_ID '1'

const char* WIFI_SSID = "isthislife?";
const char* WIFI_PASS = "Admin1234";

const uint16_t UDP_PORT = 4210;
// ==================

WiFiUDP udp;
char rxBuf[256];

void sendToLaptop(const char* msg) {
    udp.beginPacket("255.255.255.255", UDP_PORT);
    udp.print('0');     // recipient = laptop
    udp.print(msg);
    udp.endPacket();
}

void setup() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    // Wait for WiFi connection
    while (WiFi.status() != WL_CONNECTED) {
        delay(200);
    }

    // Start UDP
    udp.begin(UDP_PORT);

    // ---- CONNECTION ANNOUNCEMENT ----
    sendToLaptop("ESP32_1_CONNECTED");
}

void loop() {
    int packetSize = udp.parsePacket();
    if (packetSize <= 0) return;

    int len = udp.read(rxBuf, sizeof(rxBuf) - 1);
    if (len <= 0) return;
    rxBuf[len] = '\0';

    // Message format: <RID><PAYLOAD>
    char recipient = rxBuf[0];

    // Ignore packets not addressed to this ESP32
    if (recipient != ESP32_ID) return;

    // Echo payload back to laptop (ID 0)
    udp.beginPacket("255.255.255.255", UDP_PORT);
    udp.print('0');
    udp.print(&rxBuf[1]);   // payload
    udp.endPacket();
}
