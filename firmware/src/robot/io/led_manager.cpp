#ifdef ROLE_ROBOT

#include "led_manager.h"
#include "../config/robot_config.h"
#include <Arduino.h>

static LedMode currentMode = LED_OFF;
static uint32_t lastToggleTime = 0;
static bool physicalLedState = false;

// The active pattern currently being played
static const uint16_t* activePattern = nullptr;
static uint8_t patternIndex = 0;

// ==============================================================================
// PATTERN DEFINITIONS (Even index = ON time, Odd index = OFF time, 0 = Loop)
// ==============================================================================
const uint16_t PAT_FAST[]    = {100, 100, 0}; 
const uint16_t PAT_SLOW[]    = {500, 500, 0}; 

// 3x Short: ON 100, OFF 100, ON 100, OFF 100, ON 100, OFF 800 (Pause)
const uint16_t PAT_3_SHORT[] = {100, 100, 100, 100, 100, 800, 0};

// 3x Long: ON 500, OFF 200, ON 500, OFF 200, ON 500, OFF 800 (Pause)
const uint16_t PAT_3_LONG[]  = {500, 200, 500, 200, 500, 800, 0};

// Mix (Short-Long-Short): ON 100, OFF 100, ON 500, OFF 100, ON 100, OFF 800
const uint16_t PAT_MIX[]     = {100, 100, 500, 100, 100, 800, 0};
// ==============================================================================

void initLed() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
}

void setLedMode(LedMode mode) {
    if (currentMode == mode) return; // Do nothing if mode hasn't changed
    
    currentMode = mode;
    patternIndex = 0;
    activePattern = nullptr;

    if (mode == LED_OFF) {
        physicalLedState = false;
        digitalWrite(LED_PIN, LOW);
    } 
    else if (mode == LED_SOLID) {
        physicalLedState = true;
        digitalWrite(LED_PIN, HIGH);
    } 
    else {
        // It's a pattern mode! Assign the correct array pointer.
        switch(mode) {
            case LED_BLINK_FAST:      activePattern = PAT_FAST; break;
            case LED_BLINK_SLOW:      activePattern = PAT_SLOW; break;
            case LED_PATTERN_3_SHORT: activePattern = PAT_3_SHORT; break;
            case LED_PATTERN_3_LONG:  activePattern = PAT_3_LONG; break;
            case LED_PATTERN_MIX:     activePattern = PAT_MIX; break;
            default: break;
        }
        
        // Kickstart the first step of the pattern immediately
        if (activePattern != nullptr) {
            physicalLedState = true; // Index 0 is always an 'ON' state
            digitalWrite(LED_PIN, HIGH);
            lastToggleTime = millis();
        }
    }
}

void updateLed() {
    // Only process timing if we have an active pattern
    if (activePattern == nullptr) return; 

    uint32_t now = millis();
    uint16_t currentDuration = activePattern[patternIndex];

    if (now - lastToggleTime >= currentDuration) {
        patternIndex++; // Move to the next step

        // If we hit the 0-terminator, loop back to the beginning
        if (activePattern[patternIndex] == 0) {
            patternIndex = 0;
        }

        // Even indices (0, 2, 4...) mean the LED should be ON
        // Odd indices (1, 3, 5...) mean the LED should be OFF
        physicalLedState = (patternIndex % 2 == 0);
        digitalWrite(LED_PIN, physicalLedState);

        lastToggleTime = now;
    }
}

#endif // ROLE_ROBOT