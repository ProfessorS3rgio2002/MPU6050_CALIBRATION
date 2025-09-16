// MPU6050 Motorcycle Accident Detection - Angle Monitoring
// SCL = 22
// SDA = 21

#include <Wire.h>
#include <MPU6050.h>

#include "WiFiTCP.h"
#include "EmergencyCancel.h"
#include "IgnitionHandler.h"
#include "Config.h"
#include "MotionDetection.h"
MPU6050 mpu;
bool detectImpact();

// Motion detection
MotionDetection motionDetector(0.3);  // 0.3g threshold for movement detection

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
// float impactThreshold = 3.0; // Default impact threshold in g
// Calibration parameters
const int CALIBRATION_SAMPLES = 1000;
const int CALIBRATION_DELAY = 2; // ms between samples

// WiFi and TCP parameters
bool ignitionOn = false; 
const char* WIFI_SSID = "MPU6050_AP";
const char* WIFI_PASS = "12345678";
const uint16_t TCP_PORT = 3333;
WiFiTCP wifiTCP;

#define THEFT_RELAY_PIN 26

// Add these variables at the top with other globals
const int REQUIRED_CONSECUTIVE_FALLS = 40; // 40 checks × 50ms = 2 seconds
static int consecutiveFallCount = 0;

EmergencyCancel emergencyCancel;

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

  // Start WiFi AP and TCP server
  Serial.println("Starting WiFi Access Point and TCP server...");
  wifiTCP.begin(WIFI_SSID, WIFI_PASS, TCP_PORT);
  Serial.print("WiFi AP SSID: "); Serial.println(WIFI_SSID);
  Serial.print("WiFi AP Password: "); Serial.println(WIFI_PASS);
  Serial.print("TCP Port: "); Serial.println(TCP_PORT);
  Serial.println("Connect your phone to this WiFi and use a TCP client app to connect.");
  
  pinMode(THEFT_RELAY_PIN, OUTPUT);
  emergencyCancel.begin();
}

void loop() {
  // Update and check ignition state first
  updateIgnitionState();
  if (hasIgnitionStateChanged()) {
    float voltage = getIgnitionVoltage();
    if (ignitionOn) {
      Serial.printf("🔑 Ignition turned ON (%.2fV)\n", voltage);
    } else {
      Serial.printf("🔓 Ignition turned OFF (%.2fV)\n", voltage);
    }
  }

  static int fallCheckCount = 0; // Count how many times we check for fall after impact
  static bool impactOccurred = false; // Track if impact happened
  const int MAX_FALL_CHECKS = 60; // Check for fall only 60 times after impact

  // Read sensor data
  readMPU6050();
  
  // Calculate angles and update motion detection
  calculateAngles();
  
  // Update motion detection with accelerometer data
  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;
  motionDetector.update(ax_g, ay_g, az_g);
  
  // Check for impact first
  bool impactDetected = detectImpact();
  if (impactDetected && !impactOccurred) {
    // Only consider it a potential accident if we were moving
    if (motionDetector.isMoving()) {
      Serial.println("Vehicle was in motion before impact!");
      Serial.printf("Linear acceleration: %.2fg\n", motionDetector.getLinearAccelMagnitude());
      Serial.printf("Average speed estimate: %.2f\n", motionDetector.getAverageSpeed());
    impactOccurred = true;
    fallCheckCount = 0;
    consecutiveFallCount = 0; // Reset consecutive fall counter
    Serial.println("*** STRONG IMPACT DETECTED! ***");
    Serial.println("Checking for persistent fall...");
  }
}
  
  // Check for fall conditions
  String status = "UPRIGHT";
  String fallType = "";
  // Determine fall type
  if (roll <= FALL_THRESHOLD_LEFT) {
    fallType = "LEFT FALL";
  } else if (roll >= FALL_THRESHOLD_RIGHT) {
    fallType = "RIGHT FALL";
  }
  if (pitch >= FALL_THRESHOLD_FORWARD) {
    if (fallType != "") fallType += " + ";
    fallType += "FORWARD FALL";
  } else if (pitch <= FALL_THRESHOLD_BACKWARD) {
    if (fallType != "") fallType += " + ";
    fallType += "BACKWARD FALL";
  }

  // Set status for TCP data
  if (fallType != "") {
    status = fallType;
  } else if (abs(roll) < 10 && abs(pitch) < 10) {
    status = "UPRIGHT";
  } else if (roll < -10) {
    status = "LEANING_LEFT";
  } else if (roll > 10) {
    status = "LEANING_RIGHT";
  }
  if (pitch > 10 && fallType == "") {
    status += ",NOSE_DOWN";
  } else if (pitch < -10 && fallType == "") {
    status += ",NOSE_UP";
  }

  // Print angle data
  printAngleData();

  // Send data to TCP client (phone app)
  String tcpData = String(roll, 2) + "," + 
                   String(pitch, 2) + "," + 
                   status + "," +
                   String(motionDetector.getLinearAccelMagnitude(), 2) + "," +  // Linear acceleration in g
                   String(motionDetector.getAverageSpeed(), 2) + "," +          // Keep for compatibility
                   (motionDetector.isMoving() ? "MOVING" : "STATIONARY");

  wifiTCP.handleClient(tcpData);
  
  // Check for TCP commands
  handleTCPCommands();

  // Check for fall conditions and send fall message if needed
  if (fallType != "") {
    Serial.println("*** FALL DETECTED! ***");
    Serial.print("Fall Type: ");
    Serial.println(fallType);
    Serial.println("**********************");
    String fallMsg = "FALL_DETECTED," + fallType;
    wifiTCP.handleClient(fallMsg);
    
    // Modified fall detection logic
    if (fallType != "") {
      consecutiveFallCount++; // Increment consecutive fall counter
      Serial.print("Consecutive fall count: ");
      Serial.print(consecutiveFallCount);
      Serial.print("/");
      Serial.println(REQUIRED_CONSECUTIVE_FALLS);
      
      // Only trigger accident if fall persists for required duration and was moving
      if (impactOccurred && consecutiveFallCount >= REQUIRED_CONSECUTIVE_FALLS) {
        if (motionDetector.isMoving()) {
          Serial.println("!!! REAL ACCIDENT DETECTED !!!");
          Serial.println("Vehicle was in motion before the fall!");
        Serial.println("Motorcycle has been fallen for 2 seconds after impact!");
        wifiTCP.handleClient("ACCIDENT_ALERT," + fallType);
        emergencyPattern();
          emergencyCancel.setEmergencyActive(true);  // Start monitoring for cancel pattern
          Serial.println("Turn ignition ON/OFF 3 times to cancel emergency");
          // Don't reset flags until cancelled
        } else {
          Serial.println("False alarm - Vehicle was stationary before fall");
          impactOccurred = false;
          consecutiveFallCount = 0;
        }
      }
    } else {
      consecutiveFallCount = 0; // Reset if not fallen
    }
  }
  
  // If impact occurred, increment fall check counter
  if (impactOccurred) {
    fallCheckCount++;
    
    // Reset if we've checked enough times without finding persistent fall
    if (fallCheckCount >= MAX_FALL_CHECKS) {
      impactOccurred = false;
      fallCheckCount = 0;
      consecutiveFallCount = 0;
      Serial.println("Impact timeout - no persistent fall detected");
    }
  }

  // Update emergency cancel system
  emergencyCancel.update();
  
  // Check if emergency was cancelled
  if (emergencyCancel.checkCancelPattern()) {
      Serial.println("Emergency call cancelled by user!");
      wifiTCP.handleClient("STATUS,Emergency cancelled by user");
      impactOccurred = false;
      consecutiveFallCount = 0;
      fallCheckCount = 0;
      emergencyCancel.reset();
  }
  
  // Check for serial commands
  if (Serial.available()) {
    String serialCmd = Serial.readStringUntil('\n');
    serialCmd.trim();
    
        if (serialCmd == "simulate") {
        Serial.println("\n=== SIMULATING ACCIDENT ===");
        Serial.println("[SIM] Setting emergency active state...");
        
        // Simulate the same sequence as a real accident
        Serial.println("[SIM] !!! REAL ACCIDENT DETECTED !!!");
        Serial.println("[SIM] Simulated accident - RIGHT FALL");
        wifiTCP.handleClient("ACCIDENT_ALERT,RIGHT FALL");
        emergencyPattern();
        emergencyCancel.setEmergencyActive(true);  // Start monitoring for cancel pattern
        
        Serial.println("[SIM] Emergency active: " + String(emergencyCancel.isEmergencyActive() ? "YES" : "NO"));
        Serial.println("[SIM] Current ignition state: " + String(ignitionOn ? "ON" : "OFF"));
        Serial.println("[SIM] Turn ignition ON/OFF 3 times to cancel emergency");
        Serial.println("[SIM] You have 3 seconds to complete the pattern");
    }
  }
  
  delay(50); // Update rate: 10Hz
}


bool detectImpact() {
  // Calculate acceleration magnitude in g
  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;
  float accMag = sqrt(ax_g * ax_g + ay_g * ay_g);
  // Use the variable threshold
  // return accMag > 3.0;
  return accMag > 2.0;
}



void calibrateMPU6050() {
  Serial.println("\n=== PRE-CALIBRATION POSITION CHECK ===");
  
  // Check current position before calibration
  if (!checkUprightPosition()) {
    Serial.println("⚠️ CALIBRATION WARNING!");
    Serial.println("Motorcycle is not in upright position, but continuing calibration...");
    Serial.println("Note: Calibration may not be optimal. Position bike upright for best results.");
  } else {
    Serial.println("✅ Motorcycle position verified as UPRIGHT");
  }
  
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
bool checkUprightPosition() {
  Serial.println("Checking motorcycle position...");
  
  const int POSITION_CHECK_SAMPLES = 100;
  const float UPRIGHT_TOLERANCE = 15.0; // degrees tolerance for "upright"
  
  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  
  // Take samples to determine current position
  for (int i = 0; i < POSITION_CHECK_SAMPLES; i++) {
    int16_t temp_ax, temp_ay, temp_az, temp_gx, temp_gy, temp_gz;
    mpu.getMotion6(&temp_ax, &temp_ay, &temp_az, &temp_gx, &temp_gy, &temp_gz);
    
    ax_sum += temp_ax;
    ay_sum += temp_ay;
    az_sum += temp_az;
    
    delay(10);
  }
  
  // Calculate average accelerometer values
  float avg_ax = (ax_sum / POSITION_CHECK_SAMPLES) / 16384.0;
  float avg_ay = (ay_sum / POSITION_CHECK_SAMPLES) / 16384.0;
  float avg_az = (az_sum / POSITION_CHECK_SAMPLES) / 16384.0;
  
  // Calculate current angles
  float current_roll = atan2(avg_ay, avg_az) * 180 / PI;
  float current_pitch = atan2(-avg_ax, sqrt(avg_ay * avg_ay + avg_az * avg_az)) * 180 / PI;
  
  // Display current position
  Serial.print("Current Position - Roll: ");
  Serial.print(current_roll, 1);
  Serial.print("°, Pitch: ");
  Serial.print(current_pitch, 1);
  Serial.println("°");
  
  // Check if motorcycle is reasonably upright
  if (abs(current_roll) <= UPRIGHT_TOLERANCE && abs(current_pitch) <= UPRIGHT_TOLERANCE) {
    Serial.print("Position Status: UPRIGHT (within ±");
    Serial.print(UPRIGHT_TOLERANCE, 0);
    Serial.println("° tolerance)");
    return true;
  } else {
    Serial.println("Position Status: NOT UPRIGHT");
    
    // Provide specific guidance
    if (abs(current_roll) > UPRIGHT_TOLERANCE) {
      if (current_roll > 0) {
        Serial.println("🏍️  Motorcycle is leaning RIGHT - Please straighten it");
      } else {
        Serial.println("🏍️  Motorcycle is leaning LEFT - Please straighten it");
      }
    }
    
    if (abs(current_pitch) > UPRIGHT_TOLERANCE) {
      if (current_pitch > 0) {
        Serial.println("🏍️  Motorcycle nose is DOWN - Please level it");
      } else {
        Serial.println("🏍️  Motorcycle nose is UP - Please level it");
      }
    }
    
    return false;
  }
}

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

void performManualCalibration() {
  // Alternative calibration function that can be called manually
  // Useful if you want to recalibrate without restarting
  Serial.println("\n=== MANUAL RECALIBRATION ===");
  if (checkUprightPosition()) {
    calibrateMPU6050();
    Serial.println("Manual calibration completed!");
  } else {
    Serial.println("Cannot calibrate - motorcycle not in upright position");
  }
}

void printCalibrationStatus() {
  Serial.println("\n=== CURRENT CALIBRATION OFFSETS ===");
  Serial.print("Accelerometer - X: "); Serial.print(ax_offset);
  Serial.print(", Y: "); Serial.print(ay_offset);
  Serial.print(", Z: "); Serial.println(az_offset);
  Serial.print("Gyroscope - X: "); Serial.print(gx_offset);
  Serial.print(", Y: "); Serial.print(gy_offset);
  Serial.print(", Z: "); Serial.println(gz_offset);
  Serial.println("=====================================");
}


void emergencyPattern() {
  Serial.println("Starting emergency pattern...");
  
  // Phase 1: Rapid emergency sirens (3 cycles) - Shorter delays
  for (int cycle = 0; cycle < 3; cycle++) {
    // Fast ascending siren
    for (int i = 0; i < 8; i++) {
      digitalWrite(THEFT_RELAY_PIN, HIGH);
      Serial.print("🚑");
      delay(30 + (i * 10)); // Reduced delay 30-100ms
      digitalWrite(THEFT_RELAY_PIN, LOW);
      delay(20); // Reduced delay
      yield(); // Allow ESP32 to handle WiFi tasks
    }
    
    // Fast descending siren
    for (int i = 7; i >= 0; i--) {
      digitalWrite(THEFT_RELAY_PIN, HIGH);
      Serial.print("🚨");
      delay(30 + (i * 10)); // Reduced delay 100-30ms
      digitalWrite(THEFT_RELAY_PIN, LOW);
      delay(20); // Reduced delay
      yield(); // Allow ESP32 to handle WiFi tasks
    }
    yield(); // Allow ESP32 to handle WiFi tasks
  }
  
  delay(200); // Reduced pause
  
  // Phase 2: Urgent triple bursts (3 sets) - Reduced from 5 sets
  for (int set = 0; set < 3; set++) {
    // Triple burst
    for (int burst = 0; burst < 3; burst++) {
      digitalWrite(THEFT_RELAY_PIN, HIGH);
      Serial.print("⚡");
      delay(80); // Reduced delay
      digitalWrite(THEFT_RELAY_PIN, LOW);
      delay(80); // Reduced delay
      yield(); // Allow ESP32 to handle WiFi tasks
    }
    delay(200); // Reduced pause between sets
    yield(); // Allow ESP32 to handle WiFi tasks
  }
  
  delay(200); // Reduced pause
  
  // Phase 3: Final urgent pulses (2 pulses) - Reduced from 4
  for (int i = 0; i < 2; i++) {
    digitalWrite(THEFT_RELAY_PIN, HIGH);
    Serial.print("🆘");
    delay(400); // Reduced long pulse
    digitalWrite(THEFT_RELAY_PIN, LOW);
    delay(100); // Reduced pause
    yield(); // Allow ESP32 to handle WiFi tasks
  }
  
  Serial.println(" EMERGENCY PATTERN COMPLETE");
}

void handleTCPCommands() {
  // Check if there's an incoming command from TCP client
  String command = wifiTCP.getCommand();
  if (command.length() > 0) {
    command.trim();
    command.toLowerCase();
    
    Serial.print("Received TCP command: ");
    Serial.println(command);
    
    if (command == "recalibrate") {
      Serial.println("TCP Command: Recalibrating device...");
      wifiTCP.handleClient("STATUS,Recalibrating device...");
      calibrateMPU6050();
      wifiTCP.handleClient("STATUS,Recalibration complete!");
      Serial.println("TCP Command: Recalibration complete!");
    }
    else if (command == "restart") {
      Serial.println("TCP Command: Restarting device...");
      wifiTCP.handleClient("STATUS,Restarting device...");
      delay(1000);
      ESP.restart();
    }
    else if (command == "alarm") {
      Serial.println("TCP Command: Manual alarm triggered!");
      wifiTCP.handleClient("STATUS,Manual alarm activated!");
      emergencyPattern(); // Activate emergency siren/relay
      wifiTCP.handleClient("STATUS,Manual alarm complete!");
      Serial.println("TCP Command: Manual alarm complete!");
    }
    else if (command.startsWith("setthreshold,")) {
      // Extract the threshold value
      // String thresholdStr = command.substring(13); // Remove "setthreshold,"
      // float newThreshold = thresholdStr.toFloat();
      // if (newThreshold > 0 && newThreshold <= 10.0) { // Reasonable range
      //   impactThreshold = newThreshold;
      //   Serial.print("TCP Command: Impact threshold set to ");
      //   Serial.println(impactThreshold);
      //   wifiTCP.handleClient("STATUS,Impact threshold set to " + String(impactThreshold, 1) + "g");
      // } else {
      //   Serial.println("TCP Command: Invalid threshold value");
      //   wifiTCP.handleClient("ERROR,Invalid threshold value: " + thresholdStr);
      // }
    }
    else {
      Serial.print("TCP Command: Unknown command - ");
      Serial.println(command);
      wifiTCP.handleClient("ERROR,Unknown command: " + command);
    }
  }
}