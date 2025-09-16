#include "MotionDetection.h"

MotionDetection::MotionDetection(float threshold) : 
    historyIndex(0),
    movementThreshold(threshold),
    linearAccelMag(0),
    avgSpeed(0),
    moving(false) {
    memset(accelHistory, 0, sizeof(accelHistory));
}

void MotionDetection::update(float ax, float ay, float az) {
    // Calculate linear acceleration magnitude
    // We subtract 1g (9.81 m/s²) from the vertical component to remove gravity
    float gravity_z = 1.0;  // Assuming z-axis is vertical in normal orientation
    float linear_ax = ax;
    float linear_ay = ay;
    float linear_az = az - gravity_z;
    
    // Calculate magnitude of linear acceleration
    linearAccelMag = sqrt(linear_ax * linear_ax + linear_ay * linear_ay + linear_az * linear_az);
    
    // Store in circular buffer
    accelHistory[historyIndex] = linearAccelMag;
    historyIndex = (historyIndex + 1) % HISTORY_SIZE;
    
    updateMovingState();
    updateAverageSpeed();
}

void MotionDetection::updateMovingState() {
    // Count how many recent samples are above threshold
    int activeCount = 0;
    for (int i = 0; i < HISTORY_SIZE; i++) {
        if (accelHistory[i] > movementThreshold) {
            activeCount++;
        }
    }
    
    // Consider moving if more than 25% of recent samples show movement
    moving = (activeCount > (HISTORY_SIZE / 4));
}

void MotionDetection::updateAverageSpeed() {
    // Simple integration of acceleration to estimate speed
    // This is a rough approximation and will drift over time
    avgSpeed = 0;
    for (int i = 0; i < HISTORY_SIZE; i++) {
        avgSpeed += accelHistory[i];
    }
    avgSpeed = avgSpeed / HISTORY_SIZE;
}

bool MotionDetection::isMoving() const {
    return moving;
}

float MotionDetection::getLinearAccelMagnitude() const {
    return linearAccelMag;
}

float MotionDetection::getAverageSpeed() const {
    return avgSpeed;
}

void MotionDetection::reset() {
    memset(accelHistory, 0, sizeof(accelHistory));
    historyIndex = 0;
    linearAccelMag = 0;
    avgSpeed = 0;
    moving = false;
}