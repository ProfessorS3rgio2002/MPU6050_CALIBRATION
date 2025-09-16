#include "IgnitionHandler.h"
#include "Config.h"

// External reference to the global variable
extern bool ignitionOn;

// Choose method: true = analog (voltage divider), false = digital
#define USE_ANALOG_IGNITION true

// Analog ignition settings (for 8V demo)
#define IGN_R1 9600.0f   // ohms (top resistor)
#define IGN_R2 4900.0f   // ohms (bottom resistor)
#define IGN_ON_THRESHOLD_V 6.0f
#define IGN_OFF_THRESHOLD_V 5.0f
// #define IGN_ON_THRESHOLD_V 7.0f
// #define IGN_OFF_THRESHOLD_V 6.0f

// Internal state
static bool ignitionLogical = false;
static float lastVoltage = 0.0f;

void initIgnitionHandler() {
    if (USE_ANALOG_IGNITION) {
        analogReadResolution(12);
        analogSetPinAttenuation(IGNITION_SWITCH, ADC_11db);
        Serial.println("[IGN] Initialized analog ignition reading on pin " + String(IGNITION_SWITCH));
    } else {
        pinMode(IGNITION_SWITCH, INPUT_PULLUP);
        Serial.println("[IGN] Initialized digital ignition reading on pin " + String(IGNITION_SWITCH));
    }
}

static float readIgnitionVoltage() {
    if (!USE_ANALOG_IGNITION) return 0.0f;
    
    uint32_t sum = 0;
    const int samples = 8;
    for (int i = 0; i < samples; i++) {
        sum += analogRead(IGNITION_SWITCH);
        delayMicroseconds(300);
    }
    float avg = sum / (float)samples;
    float vOut = (avg / 4095.0f) * 3.3f; // 12-bit ADC, 3.3V reference
    float vIn = vOut * ((IGN_R1 + IGN_R2) / IGN_R2);
    lastVoltage = vIn;
    return vIn;
}

void updateIgnitionState() {
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate < 100) return; // 10 Hz update rate
    lastUpdate = millis();

    if (USE_ANALOG_IGNITION) {
        float voltage = readIgnitionVoltage();
        bool previousState = ignitionOn;

        // Hysteresis logic (always check both)
        if (voltage >= IGN_ON_THRESHOLD_V) {
            ignitionLogical = true;
            ignitionOn = true;
            if (previousState != ignitionOn) {
                Serial.printf("[IGN] State changed to ON (%.2f V)\n", voltage);
            }
        } else if (voltage <= IGN_OFF_THRESHOLD_V) {
            ignitionLogical = false;
            ignitionOn = false;
            if (previousState != ignitionOn) {
                Serial.printf("[IGN] State changed to OFF (%.2f V)\n", voltage);
            }
        }
        // If voltage is between thresholds, keep previous state
    } else {
        // Digital reading (LOW = ignition ON)
        bool currentReading = (digitalRead(IGNITION_SWITCH) == LOW);
        bool previousState = ignitionOn;
        ignitionLogical = currentReading;
        ignitionOn = currentReading;
        if (previousState != ignitionOn) {
            Serial.println(ignitionOn ? "[IGN] State changed to ON (digital)" : "[IGN] State changed to OFF (digital)");
        }
    }
}

bool getIgnitionState() {
    return ignitionOn;  // Return global variable
}

bool hasIgnitionStateChanged() {
    static bool lastGlobalState = false;
    bool changed = (ignitionOn != lastGlobalState);
    if (changed) {
        lastGlobalState = ignitionOn;
    }
    return changed;
}

float getIgnitionVoltage() {
    return lastVoltage;
}