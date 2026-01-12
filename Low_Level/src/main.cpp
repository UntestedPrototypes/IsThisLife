#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>
#include <Wire.h>

#include <Adafruit_BNO055.h>
#include <Adafruit_MCP9808.h>
#include <Adafruit_INA219.h>
#include <cmath>

// ================= WIFI / UDP =================
const char* WIFI_SSID = "isthislife?";
const char* WIFI_PASS = "Admin1234";
const uint16_t UDP_PORT = 4210;
#define ESP32_ID '1'

// ================= MOTOR PINS =================
#define MAIN_MOTOR_PIN 27
#define SERVO_L_PIN    25
#define SERVO_R_PIN    33

// ================= STRUCTS =================
struct Vec3 {
    float x, y, z;
    Vec3(float a=0, float b=0, float c=0) : x(a), y(b), z(c) {}
};

struct motorChannel {
    uint8_t pin;
    unsigned int min_us;
    unsigned int max_us;
    unsigned int neutral_us;
    unsigned int speed_range_us;
    unsigned int min_delta_us;
    bool direction_inverted;
    unsigned int speed;
    Vec3 pid;

    motorChannel(uint8_t pin_,
                 unsigned int min_us_,
                 unsigned int max_us_,
                 unsigned int neutral_us_,
                 unsigned int speed_range_us_,
                 unsigned int min_delta_us_ = 0,
                 bool direction_inverted_ = false,
                 unsigned int speed_ = 100,
                 Vec3 pid_ = Vec3())
        : pin(pin_),
          min_us(min_us_),
          max_us(max_us_),
          neutral_us(neutral_us_),
          speed_range_us(speed_range_us_),
          min_delta_us(min_delta_us_),
          direction_inverted(direction_inverted_),
          speed(speed_),
          pid(pid_) {}
};

// ================= MOTOR CONFIG =================
motorChannel mainMotor(MAIN_MOTOR_PIN, 1000, 2000, 1500, 500, true, 100);
motorChannel servoL   (SERVO_L_PIN,    500, 2500, 1500, 500, 75, false, 150);
motorChannel servoR   (SERVO_R_PIN,    500, 2500, 1500, 500, 75, true, 150);

// ================= SERVOS =================
Servo mainServo;
Servo servoLeft;
Servo servoRight;

// ================= UDP =================
WiFiUDP udp;
char rxBuf[128];

// ================= SENSORS =================
Adafruit_BNO055 imuR(55, 0x28);
Adafruit_BNO055 imuL(56, 0x29);
Adafruit_MCP9808 mcp;
Adafruit_INA219 ina219;

bool imuR_ok = false;
bool imuL_ok = false;
bool mcp_ok  = false;
bool ina_ok  = false;

// ================= STATE =================
float temp  = NAN;
float vBatt = NAN;

float x = 0.0f;
float y = 0.0f;

// ================= HELPERS =================
static inline int clampInt(int v, int lo, int hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

void sendToLaptop(const String& msg) {
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.print('0');
    udp.print(msg);
    udp.endPacket();
}

// ================= MOTOR CONTROL =================
void applyMotorControl(motorChannel& m, Servo& s, float input) {
    if (fabs(input) < 0.2f) {
        s.writeMicroseconds(m.neutral_us);
        return;
    }

    if (m.direction_inverted) input = -input;

    int pwm = (int)(m.neutral_us + input * m.speed);

    if (abs(pwm - (int)m.neutral_us) < (int)m.min_delta_us)
        pwm = m.neutral_us;

    pwm = clampInt(pwm, m.min_us, m.max_us);
    s.writeMicroseconds(pwm);
}

void applyXY(float x, float y) {
    applyMotorControl(mainMotor, mainServo, x);
    applyMotorControl(servoL, servoLeft, y);
    applyMotorControl(servoR, servoRight, y);
}

// ================= UDP PARSING =================
void parseUdp() {
    int packetSize = udp.parsePacket();
    if (packetSize <= 0) return;

    int len = udp.read(rxBuf, sizeof(rxBuf) - 1);
    if (len <= 0) return;

    rxBuf[len] = '\0';

    if (rxBuf[0] != ESP32_ID) return;

    sscanf(&rxBuf[1], "%f,%f", &x, &y);
}

// ================= SETUP =================
void setup() {
    Serial.begin(115200);

    mainServo.attach(MAIN_MOTOR_PIN, mainMotor.min_us, mainMotor.max_us);
    servoLeft.attach(SERVO_L_PIN, servoL.min_us, servoL.max_us);
    servoRight.attach(SERVO_R_PIN, servoR.min_us, servoR.max_us);

    mainServo.writeMicroseconds(mainMotor.neutral_us);
    servoLeft.writeMicroseconds(servoL.neutral_us);
    servoRight.writeMicroseconds(servoR.neutral_us);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) delay(200);

    udp.begin(UDP_PORT);

    Wire.begin();

    imuR_ok = imuR.begin();
    imuL_ok = imuL.begin();
    mcp_ok  = mcp.begin();
    ina_ok  = ina219.begin();

    sendToLaptop("ESP32_READY");
}

// ================= LOOP =================
void loop() {

    // ---- ALWAYS parse UDP ----
    parseUdp();

    // ---- Read sensors ----
    if (mcp_ok)  temp  = mcp.readTempC();
    if (ina_ok)  vBatt = ina219.getBusVoltage_V();

    // ---- Safety override ----
    if (!isnan(temp) && temp > 65.0f) {
        x = 0.0f;
        y = 0.0f;
    }

    if (!isnan(vBatt) && vBatt < 14.8f) {
        x = 0.0f;
        y = 0.0f;
    }

    // ---- Apply control ----
    applyXY(x, y);

    // ---- Telemetry / confirmation (INTENTIONALLY COMMENTED) ----
    String msg = "CTRL,X" + String(x, 2) +
                 ",Y" + String(y, 2) +
                 ",Temp" + String(temp, 1) +
                 ",Batt" + String(vBatt, 2);
    sendToLaptop(msg);

    // ---- Yield to WiFi task ----
    delay(1);
}
