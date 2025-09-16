#ifndef MOTION_DETECTION_H
#define MOTION_DETECTION_H

#include <Arduino.h>

class MotionDetection {
public:
    MotionDetection(float threshold = 0.3);
    void update(float ax, float ay, float az);  // Remove GPS parameter
    bool isMoving() const;
    float getLinearAccelMagnitude() const;
    float getAverageSpeed() const;
    void reset();

private:
    static const int HISTORY_SIZE = 20;
    
    void updateMovingState();
    void updateAverageSpeed();

    float accelHistory[HISTORY_SIZE];
    int historyIndex;
    float movementThreshold;
    float linearAccelMag;
    float avgSpeed;
    bool moving;
};

#endif