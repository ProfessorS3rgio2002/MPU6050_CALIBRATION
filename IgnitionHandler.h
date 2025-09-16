#ifndef IGNITION_HANDLER_H
#define IGNITION_HANDLER_H

#include <Arduino.h>

// Initialize ignition handler (sets up ADC if using analog)
void initIgnitionHandler();

// Update ignition state (call this in loop)
void updateIgnitionState();

// Get current ignition state
bool getIgnitionState();

// Check if ignition state changed since last call
bool hasIgnitionStateChanged();

// Get last measured voltage (if using analog)
float getIgnitionVoltage();

#endif