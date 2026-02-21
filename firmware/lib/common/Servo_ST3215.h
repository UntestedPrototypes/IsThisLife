#ifdef ROLE_TESTING
#ifndef SERVO_ST3215_H
#define SERVO_ST3215_H

#include <Arduino.h>
#include "SCServo.h" 

class Servo_ST3215 {
private:
    SMS_STS st;
    int id1, id2;
    int accel;
    int torqueThreshold;
    int slowdownThreshold = 500; // Distance from limit at which to start slowing down

    // Position Tracking Variables
    long rawTruePos1, rawTruePos2;       // The software-tracked absolute positions
    int lastRaw1, lastRaw2;              // Previous raw values for wrap detection
    long zeroOffset1, zeroOffset2;       // The user-defined zero point
    int currentVelCommand;               // Tracks requested velocity

    // Limits & Configuration
    long minLimit;
    long maxLimit;
    bool reverse2;
    bool safeMode;

    void trackWraps(int servoNum, int newRaw);
public:
    Servo_ST3215(int servoID1, int servoID2);
    bool begin(HardwareSerial& serialPort, int rx, int tx);
    
    // Safety & Monitoring
    void update(); // MUST be called frequently in loop()
    bool isSystemSafe() { return !safeMode; }
    void resetSafety();
    void setSafetyThreshold(int threshold);
    void setMaxTorque(int limit);
    void emergencyStop();

    // Manual Calibration
    void disableMotors();   // Disables torque so you can move it by hand
    void enableMotors();    // Re-enables torque
    void setMinLimitToCurrentPosition(); // Saves the current position as the Min Limit
    void setMaxLimitToCurrentPosition(); // Saves the current position as the Max Limit

    // Configuration
    void setAcceleration(int a);
    void setReverseSecond(bool rev);
    void resetPositionToZero();
    void setOuterLimits(long minLim, long maxLim);
    int GetMode(int id);
    
    // Movement Commands
    void setVelocity(int targetVelocity);
    void stop(); // Immediately halt the motors

    // Feedback
    long getPosition(int id);
};

#endif
#endif // ROLE_TESTING