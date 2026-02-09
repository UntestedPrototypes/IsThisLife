#ifdef ROLE_ROBOT

#include "motors.h"
#include <Arduino.h>

void setMotors(int8_t vx, int8_t vy, int8_t omega) {
    //Serial.printf("DEBUG: Motors set - vx=%d vy=%d omega=%d\n", vx, vy, omega);
}

void stopMotors() { 
    //Serial.println("DEBUG: Motors stopped"); 
}
#endif // ROLE_ROBOT