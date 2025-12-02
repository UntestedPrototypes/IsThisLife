#include <Arduino.h>
#include <ESP32Servo.h>

ESP32PWM servo1;
int servo1_pin = 13;
int servo1_freq = 50;

ESP32PWM servo2;
int servo2_pin = 13;
int servo2_freq = 50;

ESP32PWM servo3;
int servo3_pin = 13;
int servo3_freq = 50;

// put function declarations here:
int myFunction(int, int);

void setup() {
  Serial.begin(115200);
  servo1.attachPin(servo1_pin, freq, 8);
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}