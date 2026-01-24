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
const char* WIFI_SSID = "CCC";
const char* WIFI_PASS = "$4g5$Mhr9MKb";
IPAddress broadcastIP(255, 255, 255, 255);
const uint16_t UDP_PORT = 4210;

#define ESP32_ID '1'

// ================= I2C =================
#define SDA_PIN 21
#define SCL_PIN 22

// ================= MOTOR PINS =================
#define MAIN_MOTOR_PIN 27
#define SERVO_L_PIN    25
#define SERVO_R_PIN    33

// ================= STRUCTS =================
struct Vec3 { float x, y, z; };

struct motorChannel {
  uint8_t pin;
  unsigned int min_us;
  unsigned int max_us;
  unsigned int neutral_us;
  unsigned int speed;
  unsigned int min_delta_us;
  bool inverted;
};

// ================= MOTOR CONFIG =================
motorChannel mainMotor {MAIN_MOTOR_PIN, 1000, 2000, 1500, 100, 0,  true};
motorChannel servoL    {SERVO_L_PIN,     500, 2500, 1500, 150, 75, false};
motorChannel servoR    {SERVO_R_PIN,     500, 2500, 1500, 150, 75, true};

// ================= SERVOS =================
Servo mainServo, servoLeft, servoRight;

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
Vec3 anglesR {NAN, NAN, NAN};
Vec3 anglesL {NAN, NAN, NAN};

// ================= HELPERS =================
static inline int clampInt(int v, int lo, int hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

// ================= UDP OUTPUT =================
static void sendToLaptop(const String& msg) {
  udp.beginPacket(broadcastIP, UDP_PORT);
  udp.print(0);;
  udp.print(ESP32_ID);
  udp.print(msg);
  udp.endPacket();
}

// ================= I2C CHECK =================
static bool checkI2CDevicesAndReport() {
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

  if (all_ok) sendToLaptop("I2C,OK");
  else        sendToLaptop(msg);

  return all_ok;
}

// ================= IMU MODE WATCHDOG =================
static void ensureIMUActive(Adafruit_BNO055& imu, bool ok) {
  if (!ok) return;

  if (imu.getMode() != OPERATION_MODE_NDOF) {
    imu.setMode(OPERATION_MODE_CONFIG);
    delay(10);
    imu.setExtCrystalUse(true);
    imu.setMode(OPERATION_MODE_NDOF);
    delay(20);
  }
}

// ================= IMU READ =================
static void readIMUs() {
  if (imuR_ok) {
    imu::Vector<3> e = imuR.getVector(Adafruit_BNO055::VECTOR_EULER);
    anglesR.x = e.x();
    anglesR.y = e.y();
    anglesR.z = e.z();
  }

  if (imuL_ok) {
    imu::Vector<3> e = imuL.getVector(Adafruit_BNO055::VECTOR_EULER);
    anglesL.x = e.x();
    anglesL.y = e.y();
    anglesL.z = e.z();
  }
}

// ================= MOTOR CONTROL =================
static void applyMotor(motorChannel& m, Servo& s, float in) {
  if (fabsf(in) < 0.2f) {
    s.writeMicroseconds(m.neutral_us);
    return;
  }

  if (m.inverted) in = -in;

  int pwm = m.neutral_us + (int)(in * m.speed);
  if (abs(pwm - (int)m.neutral_us) < (int)m.min_delta_us)
    pwm = m.neutral_us;

  s.writeMicroseconds(clampInt(pwm, m.min_us, m.max_us));
}

static void applyXY(float xin, float yin) {
  applyMotor(mainMotor, mainServo, xin);
  applyMotor(servoL, servoLeft, yin);
  applyMotor(servoR, servoRight, yin);
}

// ================= UDP INPUT =================
static void parseUdp() {
  int sz = udp.parsePacket();
  if (sz <= 0) return;

  int len = udp.read(rxBuf, sizeof(rxBuf) - 1);
  if (len <= 0) return;

  rxBuf[len] = 0;
  if (rxBuf[0] != ESP32_ID) return;

  sscanf(&rxBuf[1], "%f,%f", &x, &y);
}

// ================= WIFI WATCHDOG =================
static uint32_t lastWifiOkMs = 0;

static void wifiWatchdog() {
  if (WiFi.status() == WL_CONNECTED) {
    lastWifiOkMs = millis();
  } else if (millis() - lastWifiOkMs > 3000) {
    ESP.restart();
  }
}

// ================= SETUP =================
void setup() {
  delay(300);

  Serial.begin(115200);
  Serial.println("BOOT");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  // ---- WiFi ----
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - t0 > 8000) ESP.restart();
    delay(100);
  }

  WiFi.setSleep(false);
  udp.begin(UDP_PORT);
  lastWifiOkMs = millis();

  // ---- Sensors ----
  imuR_ok = imuR.begin();
  imuL_ok = imuL.begin();
  mcp_ok  = mcp.begin();
  ina_ok  = ina219.begin();

  if (imuR_ok) {
    imuR.setMode(OPERATION_MODE_CONFIG);
    delay(10);
    imuR.setExtCrystalUse(true);
    imuR.setMode(OPERATION_MODE_NDOF);
  }

  if (imuL_ok) {
    imuL.setMode(OPERATION_MODE_CONFIG);
    delay(10);
    imuL.setExtCrystalUse(true);
    imuL.setMode(OPERATION_MODE_NDOF);
  }

  // REQUIRED stabilization delay for dual IMUs
  delay(400);

  // ---- Servos ----
  mainServo.attach(MAIN_MOTOR_PIN, mainMotor.min_us, mainMotor.max_us);
  servoLeft.attach(SERVO_L_PIN, servoL.min_us, servoL.max_us);
  servoRight.attach(SERVO_R_PIN, servoR.min_us, servoR.max_us);

  mainServo.writeMicroseconds(mainMotor.neutral_us);
  servoLeft.writeMicroseconds(servoL.neutral_us);
  servoRight.writeMicroseconds(servoR.neutral_us);

  sendToLaptop("ESP32_READY");
  checkI2CDevicesAndReport();
}
IMUANg
// ================= LOOP =================
void loop() {
  wifiWatchdog();
  parseUdp();

  ensureIMUActive(imuR, imuR_ok);
  ensureIMUActive(imuL, imuL_ok);
  readIMUs();

  if (mcp_ok)  temp  = mcp.readTempC();
  if (ina_ok)  vBatt = ina219.getBusVoltage_V();

  if (!isnan(temp)  && temp  > 65.0f) { x = 0; y = 0; }
  if (!isnan(vBatt) && vBatt < 14.8f) { x = 0; y = 0; }

  applyXY(x, y);

  static uint32_t lastTx = 0;
  if (millis() - lastTx >= 1000) {
    lastTx = millis();


    sendToLaptop(
      "IMU,R," + String(anglesR.x,1) + "," +
                 String(anglesR.y,1) + "," +
                 String(anglesR.z,1) +
      ",L,"     + String(anglesL.x,1) + "," +
                 String(anglesL.y,1) + "," +
                 String(anglesL.z,1)
    );
  }

  delay(2);
}
