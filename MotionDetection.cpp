#include "MotionDetection.h"

MotionDetection::MotionDetection(float threshold) : 
    historyIndex(0),
    movementThreshold(threshold),
    linearAccelMag(0),
    avgSpeed(0),
    moving(false) {
    memset(accelHistory, 0, sizeof(accelHistory));
}

void MotionDetection::updateMovingState() {
    // Count how many recent samples are above threshold
    int activeCount = 0;
    float sum = 0;
    float variance = 0;
    
    // Calculate mean
    for (int i = 0; i < HISTORY_SIZE; i++) {
        sum += accelHistory[i];
    }
    float mean = sum / HISTORY_SIZE;
    
    // Calculate variance and check for sustained motion pattern
    int consecutiveCount = 0;
    int maxConsecutive = 0;
    float maxAccel = 0;
    bool hasSuddenSpike = false;
    
    for (int i = 0; i < HISTORY_SIZE; i++) {
        variance += (accelHistory[i] - mean) * (accelHistory[i] - mean);
        
        // Track maximum acceleration to detect sudden spikes
        if (accelHistory[i] > maxAccel) {
            maxAccel = accelHistory[i];
        }
        
        // Check for consistent motion within normal riding range (0.15 - 0.40 m/s)
        if (accelHistory[i] >= 0.13 && accelHistory[i] <= 0.40) {
            consecutiveCount++;
            maxConsecutive = max(maxConsecutive, consecutiveCount);
        } else {
            consecutiveCount = 0;
        }
        
        // Detect sudden acceleration spikes typical of falls
        if (accelHistory[i] > 0.8) { // Threshold for sudden movements
            hasSuddenSpike = true;
        }
    }
    variance /= HISTORY_SIZE;
    
    // New conditions for movement detection:
    // 1. Must have sustained motion in normal riding range
    // 2. Variance should be moderate (not too stable, not too chaotic)
    // 3. No sudden acceleration spikes
    const float MIN_VARIANCE = 0.001;    // Minimum variance for real movement
    const float MAX_VARIANCE = 0.05;     // Maximum variance for normal riding
    const float MIN_CONSECUTIVE = HISTORY_SIZE * 0.3; // Need 30% consistent samples
    
    bool hasNormalMotion = maxConsecutive >= MIN_CONSECUTIVE;
    bool hasReasonableVariance = variance >= MIN_VARIANCE && variance <= MAX_VARIANCE;
    
    // Only consider it moving if we have normal motion patterns without sudden spikes
    moving = hasNormalMotion && hasReasonableVariance && !hasSuddenSpike;

    // Enhanced debug output
    Serial.printf("Motion Debug - Accel: %.3fg, Mean: %.3f, Var: %.3f, MaxAccel: %.3f, Consec: %d/%d, Spike: %s, Moving: %s\n", 
                 linearAccelMag, mean, variance, maxAccel, maxConsecutive, (int)MIN_CONSECUTIVE, 
                 hasSuddenSpike ? "YES" : "NO", moving ? "YES" : "NO");
}

void MotionDetection::update(float ax, float ay, float az) {
    // Remove gravity component from vertical axis
    float az_no_gravity = az - 1.0;
    
    // Calculate linear acceleration magnitude
    linearAccelMag = sqrt(ax * ax + ay * ay + az_no_gravity * az_no_gravity);
    
    // Store in circular buffer
    accelHistory[historyIndex] = linearAccelMag;
    historyIndex = (historyIndex + 1) % HISTORY_SIZE;
    
    updateMovingState();
    updateAverageSpeed();
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