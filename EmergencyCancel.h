#ifndef EMERGENCY_CANCEL_H
#define EMERGENCY_CANCEL_H

#include <Arduino.h>

extern bool ignitionOn; 

class EmergencyCancel {
public:
    EmergencyCancel();
    void begin();
    void update();
    bool checkCancelPattern();
    void reset();
    bool isEmergencyActive() const;
    void setEmergencyActive(bool active);

private:
    static const unsigned long PATTERN_TIMEOUT = 3000;  // 3 seconds to complete pattern
    static const int REQUIRED_TOGGLES = 6;  // 3 on/off cycles = 6 toggles
    
    bool emergencyActive;
    int toggleCount;
    unsigned long lastToggleTime;
    bool lastIgnitionState;
};

#endif