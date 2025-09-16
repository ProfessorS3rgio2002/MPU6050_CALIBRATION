#ifndef MOTION_DETECTION_H
#define MOTION_DETECTION_H

#include <Arduino.h>

class MotionDetection {
public:
    MotionDetection(float threshold = 0.3);  // threshold in g (0.3g = moderate movement)
    void update(float ax, float ay, float az);
    bool isMoving() const;
    float getLinearAccelMagnitude() const;
    float getAverageSpeed() const;
    void reset();

private:
    static const int HISTORY_SIZE = 20;  // 1 second of history at 20Hz
    float accelHistory[HISTORY_SIZE];    // Circular buffer for acceleration history
    int historyIndex;
    float movementThreshold;
    float linearAccelMag;
    float avgSpeed;
    bool moving;
    
    void updateMovingState();
    void updateAverageSpeed();
};

#endif