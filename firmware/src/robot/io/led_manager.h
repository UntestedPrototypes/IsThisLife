#ifdef ROLE_ROBOT

#ifndef LED_MANAGER_H
#define LED_MANAGER_H

#include <stdint.h>

enum LedMode {
    LED_OFF,
    LED_SOLID,
    LED_BLINK_FAST,       // Continuous fast (100ms on/off)
    LED_BLINK_SLOW,       // Continuous slow (500ms on/off)
    LED_PATTERN_3_SHORT,  // 3 quick flashes, then pause
    LED_PATTERN_3_LONG,   // 3 long flashes, then pause
    LED_PATTERN_MIX       // 1 short, 1 long, 1 short
};

void initLed();
void setLedMode(LedMode mode);
void updateLed(); 

#endif // LED_MANAGER_H
#endif // ROLE_ROBOT