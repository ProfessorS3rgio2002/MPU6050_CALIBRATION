#include "EmergencyCancel.h"
#include "IgnitionHandler.h"

extern bool ignitionOn;  // Reference to global ignition state

EmergencyCancel::EmergencyCancel() : 
    emergencyActive(false),
    toggleCount(0),
    lastToggleTime(0),
    lastIgnitionState(false)
{
}

void EmergencyCancel::begin() {
    lastIgnitionState = ignitionOn;
    lastToggleTime = millis();
}

void EmergencyCancel::update() {
    if (!emergencyActive) {
        Serial.println("[EMG] Emergency not active, ignoring toggle checks");
        return;
    }

    bool currentIgnitionState = ignitionOn;
    unsigned long currentTime = millis();

    // Check for ignition state change
    if (currentIgnitionState != lastIgnitionState) {
        toggleCount++;
        lastToggleTime = currentTime;
        lastIgnitionState = currentIgnitionState;
        
        Serial.print("[EMG] Toggle detected! Count: ");
        Serial.print(toggleCount);
        Serial.print("/");
        Serial.print(REQUIRED_TOGGLES);
        Serial.print(" (");
        Serial.print(currentIgnitionState ? "ON" : "OFF");
        Serial.println(")");
    }

    // Check if pattern is complete
    if (toggleCount >= REQUIRED_TOGGLES) {
        Serial.println("[EMG] *** EMERGENCY CANCELLED BY USER ***");
        emergencyActive = false;
        toggleCount = 0;
    }

    // Reset if timeout
    if (currentTime - lastToggleTime > PATTERN_TIMEOUT) {
        if (toggleCount > 0) {
            Serial.println("[EMG] Pattern timeout - resetting toggle count");
            toggleCount = 0;
        }
        lastToggleTime = currentTime;
    }
}

bool EmergencyCancel::checkCancelPattern() {
    return !emergencyActive && toggleCount >= REQUIRED_TOGGLES;
}

void EmergencyCancel::reset() {
    toggleCount = 0;
    lastToggleTime = millis();
}

bool EmergencyCancel::isEmergencyActive() const {
    return emergencyActive;
}

void EmergencyCancel::setEmergencyActive(bool active) {
    emergencyActive = active;
    if (active) {
        reset();
    }
}