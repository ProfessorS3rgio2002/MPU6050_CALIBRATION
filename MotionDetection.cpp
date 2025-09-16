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
    bool hasSustainedFastMotion = false;
    int fastMotionCount = 0;
    
    for (int i = 0; i < HISTORY_SIZE; i++) {
        variance += (accelHistory[i] - mean) * (accelHistory[i] - mean);
        
        // Track maximum acceleration
        if (accelHistory[i] > maxAccel) {
            maxAccel = accelHistory[i];
        }
        
        // Check for three types of motion:
        // 1. Normal riding range (0.13 - 0.40)
        // 2. Fast riding range (0.40 - 0.80)
        // 3. Sudden spikes (> 0.80)
        if (accelHistory[i] >= 0.13 && accelHistory[i] <= 0.40) {
            consecutiveCount++;
            maxConsecutive = max(maxConsecutive, consecutiveCount);
        } else if (accelHistory[i] > 0.40 && accelHistory[i] <= 0.80) {
            fastMotionCount++;
            if (fastMotionCount >= HISTORY_SIZE * 0.2) { // 20% of samples show fast motion
                hasSustainedFastMotion = true;
            }
        } else if (accelHistory[i] > 0.80) {
            // Only count as spike if it's a sudden peak
            bool isSuddenPeak = (i > 0 && i < HISTORY_SIZE-1) && 
                               (accelHistory[i] > accelHistory[i-1] * 2.0) && 
                               (accelHistory[i] > accelHistory[i+1] * 2.0);
            if (isSuddenPeak) {
                hasSuddenSpike = true;
            }
        }
        
        if (accelHistory[i] < 0.13) {
            consecutiveCount = 0;
        }
    }
    variance /= HISTORY_SIZE;
    
    // Adjusted movement detection conditions
    const float MIN_VARIANCE = 0.001;
    const float MAX_VARIANCE = 0.05;
    const float MIN_CONSECUTIVE = HISTORY_SIZE * 0.3;
    
    bool hasNormalMotion = maxConsecutive >= MIN_CONSECUTIVE;
    bool hasReasonableVariance = variance >= MIN_VARIANCE && variance <= MAX_VARIANCE;
    
    // Consider moving if either normal motion or sustained fast motion is detected
    moving = (hasNormalMotion || hasSustainedFastMotion) && 
             !hasSuddenSpike;

    // Enhanced debug output
    Serial.printf("Motion Debug - Accel: %.3fg, Mean: %.3f, Var: %.3f, MaxAccel: %.3f, Normal: %d/%d, Fast: %d, Spike: %s, Moving: %s\n", 
                 linearAccelMag, mean, variance, maxAccel, maxConsecutive, (int)MIN_CONSECUTIVE, 
                 fastMotionCount, hasSuddenSpike ? "YES" : "NO", moving ? "YES" : "NO");
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