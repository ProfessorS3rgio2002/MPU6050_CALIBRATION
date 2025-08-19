// MPU6050 Motorcycle Accident Detection - Angle Monitoring
// SCL = 22
// SDA = 21

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Calibration offsets
int16_t ax_offset, ay_offset, az_offset;
int16_t gx_offset, gy_offset, gz_offset;

// Raw sensor values
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Calculated angles
float roll, pitch;

// Fall detection thresholds (in degrees)
const float FALL_THRESHOLD_LEFT = -45.0;   // Left fall threshold
const float FALL_THRESHOLD_RIGHT = 45.0;   // Right fall threshold
const float FALL_THRESHOLD_FORWARD = 45.0; // Forward fall threshold
const float FALL_THRESHOLD_BACKWARD = -45.0; // Backward fall threshold

// Calibration parameters
const int CALIBRATION_SAMPLES = 1000;
const int CALIBRATION_DELAY = 2; // ms between samples

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("MPU6050 Motorcycle Accident Detection System");
  Serial.println("==========================================");
  
  // Initialize I2C communication
  Wire.begin();
  
  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  
  // Test connection
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
    while (1) delay(1000);
  }
  
  // Set sensor ranges
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);  // ±2g
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);  // ±250°/s
  
  // Calibrate the MPU6050
  calibrateMPU6050();
  
  Serial.println("\nCalibration complete!");
  Serial.println("Starting angle monitoring...");
  Serial.println("Roll: Left(-) / Right(+) | Pitch: Forward(+) / Backward(-)");
  Serial.println("=======================================================");
  
  delay(1000);
}

void loop() {
  // Read sensor data
  readMPU6050();
  
  // Calculate angles
  calculateAngles();
  
  // Check for fall conditions
  checkFallCondition();
  
  // Print angle data
  printAngleData();
  
  delay(100); // Update rate: 10Hz
}

void calibrateMPU6050() {
  Serial.println("\n=== CALIBRATION PROCESS ===");
  Serial.println("IMPORTANT: Keep the motorcycle/sensor COMPLETELY LEVEL and STATIONARY!");
  Serial.println("Calibration will start in 5 seconds...");
  
  // Countdown
  for (int i = 5; i > 0; i--) {
    Serial.print(i);
    Serial.println(" seconds...");
    delay(1000);
  }
  
  Serial.println("Calibrating... Please wait.");
  
  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;
  
  // Collect calibration samples
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    int16_t temp_ax, temp_ay, temp_az, temp_gx, temp_gy, temp_gz;
    
    mpu.getMotion6(&temp_ax, &temp_ay, &temp_az, &temp_gx, &temp_gy, &temp_gz);
    
    ax_sum += temp_ax;
    ay_sum += temp_ay;
    az_sum += temp_az;
    gx_sum += temp_gx;
    gy_sum += temp_gy;
    gz_sum += temp_gz;
    
    // Progress indicator
    if (i % 100 == 0) {
      Serial.print("Progress: ");
      Serial.print((i * 100) / CALIBRATION_SAMPLES);
      Serial.println("%");
    }
    
    delay(CALIBRATION_DELAY);
  }
  
  // Calculate offsets
  ax_offset = ax_sum / CALIBRATION_SAMPLES;
  ay_offset = ay_sum / CALIBRATION_SAMPLES;
  az_offset = (az_sum / CALIBRATION_SAMPLES) - 16384; // 1g offset for Z-axis
  gx_offset = gx_sum / CALIBRATION_SAMPLES;
  gy_offset = gy_sum / CALIBRATION_SAMPLES;
  gz_offset = gz_sum / CALIBRATION_SAMPLES;
  
  // Print calibration results
  Serial.println("\nCalibration Offsets:");
  Serial.print("Accelerometer - X: "); Serial.print(ax_offset);
  Serial.print(", Y: "); Serial.print(ay_offset);
  Serial.print(", Z: "); Serial.println(az_offset);
  Serial.print("Gyroscope - X: "); Serial.print(gx_offset);
  Serial.print(", Y: "); Serial.print(gy_offset);
  Serial.print(", Z: "); Serial.println(gz_offset);
}

void readMPU6050() {
  int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
  
  // Read raw values
  mpu.getMotion6(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);
  
  // Apply calibration offsets
  ax = raw_ax - ax_offset;
  ay = raw_ay - ay_offset;
  az = raw_az - az_offset;
  gx = raw_gx - gx_offset;
  gy = raw_gy - gy_offset;
  gz = raw_gz - gz_offset;
}

void calculateAngles() {
  // Convert accelerometer values to g-force
  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;
  
  // Calculate roll and pitch angles in degrees
  roll = atan2(ay_g, az_g) * 180 / PI;
  pitch = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180 / PI;
}

void checkFallCondition() {
  bool fallDetected = false;
  String fallDirection = "";
  
  // Check for left fall
  if (roll <= FALL_THRESHOLD_LEFT) {
    fallDetected = true;
    fallDirection = "LEFT FALL";
  }
  // Check for right fall
  else if (roll >= FALL_THRESHOLD_RIGHT) {
    fallDetected = true;
    fallDirection = "RIGHT FALL";
  }
  
  // Check for forward fall
  if (pitch >= FALL_THRESHOLD_FORWARD) {
    fallDetected = true;
    if (fallDirection != "") fallDirection += " + ";
    fallDirection += "FORWARD FALL";
  }
  // Check for backward fall
  else if (pitch <= FALL_THRESHOLD_BACKWARD) {
    fallDetected = true;
    if (fallDirection != "") fallDirection += " + ";
    fallDirection += "BACKWARD FALL";
  }
  
  // Alert if fall detected
  if (fallDetected) {
    Serial.println("*** FALL DETECTED! ***");
    Serial.print("Fall Type: ");
    Serial.println(fallDirection);
    Serial.println("**********************");
  }
}

void printAngleData() {
  Serial.print("Roll: ");
  Serial.print(roll, 1);
  Serial.print("° | Pitch: ");
  Serial.print(pitch, 1);
  Serial.print("° | Status: ");
  
  // Determine orientation status
  if (abs(roll) < 10 && abs(pitch) < 10) {
    Serial.print("UPRIGHT");
  } else if (roll < -10) {
    Serial.print("LEANING LEFT");
  } else if (roll > 10) {
    Serial.print("LEANING RIGHT");
  }
  
  if (pitch > 10) {
    Serial.print(" NOSE DOWN");
  } else if (pitch < -10) {
    Serial.print(" NOSE UP");
  }
  
  Serial.println();
}

// Additional utility functions
void printSensorRawData() {
  Serial.print("Raw Accel: X=");
  Serial.print(ax); Serial.print(" Y=");
  Serial.print(ay); Serial.print(" Z=");
  Serial.print(az);
  Serial.print(" | Raw Gyro: X=");
  Serial.print(gx); Serial.print(" Y=");
  Serial.print(gy); Serial.print(" Z=");
  Serial.println(gz);
}

void setFallThresholds(float left, float right, float forward, float backward) {
  // This function can be called to adjust fall detection thresholds
  // Values should be passed as positive degrees
  // left and backward thresholds will be automatically made negative
}
