/*
 * Robot Arm Control using ESP32 and PCA9685 PWM Controller
 * Uses inverse kinematics and ramp functions for smooth motion
 * Adjusted servo positions -90 degrees from default
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// Create a PCA9685 PWM servo driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define servo min and max pulse lengths
// ADJUST THESE VALUES based on your specific servos!
#define SERVO_MIN_PULSE  150 // Minimum pulse length count (out of 4096)
#define SERVO_MAX_PULSE  600 // Maximum pulse length count (out of 4096)

// Define servo angles
#define SERVO_MIN_ANGLE  0
#define SERVO_MAX_ANGLE  180

// Define servos
#define BASE_SERVO       0  // Base rotation
#define SHOULDER_SERVO   1  // Shoulder joint
#define ELBOW_SERVO      2  // Elbow joint
#define WRIST_SERVO      3  // Wrist pitch
#define GRIPPER_SERVO    4  // Gripper

// Updated arm dimensions (in cm)
const float SHOULDER_LENGTH = 15.0;
const float ELBOW_LENGTH = 15.0;
const float WRIST_LENGTH = 13.0;

// Home position (in degrees) - adjusted by -90 degrees
const float HOME_BASE = 0.0;       // Changed from 90 to 0
const float HOME_SHOULDER = 0.0;   // Changed from 90 to 0
const float HOME_ELBOW = 0.0;      // Changed from 90 to 0
const float HOME_WRIST = 0.0;      // Changed from 90 to 0

// Current angles (in degrees)
float currentBase = HOME_BASE;
float currentShoulder = HOME_SHOULDER;
float currentElbow = HOME_ELBOW;
float currentWrist = HOME_WRIST;

// Gripper positions
const int GRIPPER_OPEN = 10;     // Adjusted for -90 degree offset
const int GRIPPER_CLOSED = -80;  // Adjusted for -90 degree offset

// Task positions (in cm) - Adjust these for your setup!
const float BEAKER_X = 20.0;
const float BEAKER_Y = 5.0;
const float BEAKER_Z = 5.0;

const float GLASS_X = 10.0;
const float GLASS_Y = 15.0;
const float GLASS_Z = 5.0;

// Ramp implementation for smooth motion
class Ramp {
  private:
    float startValue;
    float targetValue;
    float currentValue;
    unsigned long startTime;
    unsigned long duration;
    bool isActive;

  public:
    Ramp() {
      startValue = 0;
      targetValue = 0;
      currentValue = 0;
      startTime = 0;
      duration = 1000; // Default 1 second
      isActive = false;
    }

    void start(float start, float target, unsigned long durationMs) {
      startValue = start;
      targetValue = target;
      currentValue = start;
      duration = durationMs;
      startTime = millis();
      isActive = true;
    }

    float update() {
      if (!isActive) return currentValue;
      
      unsigned long currentTime = millis();
      unsigned long elapsedTime = currentTime - startTime;
      
      if (elapsedTime >= duration) {
        currentValue = targetValue;
        isActive = false;
        return currentValue;
      }
      
      // Calculate smooth S-curve (ease in/out)
      float progress = (float)elapsedTime / duration;
      float smoothProgress = 0.5 - 0.5 * cos(PI * progress);
      currentValue = startValue + (targetValue - startValue) * smoothProgress;
      
      return currentValue;
    }

    bool isDone() {
      return !isActive;
    }
};

// Create ramp objects for each servo
Ramp baseRamp;
Ramp shoulderRamp;
Ramp elbowRamp;
Ramp wristRamp;

void setup() {
  Serial.begin(115200);
  Serial.println("Robot Arm Control System");
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize the PCA9685
  pwm.begin();
  pwm.setPWMFreq(50);  // Standard servo frequency (50Hz)
  
  delay(10);
  
  // Initialize to home position
  delay(500);
  moveToHome();
  delay(1000);
}

void loop() {
  // Main program - execute water pouring task once
  pourWaterTask();
  
  // Wait before repeating
  delay(5000);
}

// Map angle in degrees to pulse width for PCA9685
// Adjusted to handle -90 to +90 degree range instead of 0 to 180
int angleToPulse(float angle) {
  // Add 90 degrees to shift the -90 to +90 range to 0 to 180
  float adjustedAngle = angle + 90.0;
  
  // Constrain to valid range
  adjustedAngle = constrain(adjustedAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  
  // Map adjusted angle to pulse length
  return map(adjustedAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
}

// Set servo position directly
void setServo(uint8_t servoNum, float angle) {
  int pulse = angleToPulse(angle);
  pwm.setPWM(servoNum, 0, pulse);
}

// Move all servos to specified angles
void moveServos(float baseAngle, float shoulderAngle, float elbowAngle, float wristAngle) {
  setServo(BASE_SERVO, baseAngle);
  setServo(SHOULDER_SERVO, shoulderAngle);
  setServo(ELBOW_SERVO, elbowAngle);
  setServo(WRIST_SERVO, wristAngle);
  
  // Update current positions
  currentBase = baseAngle;
  currentShoulder = shoulderAngle;
  currentElbow = elbowAngle;
  currentWrist = wristAngle;
}

// Move servos with ramping for smooth motion
void moveServosSmooth(float baseAngle, float shoulderAngle, float elbowAngle, float wristAngle, unsigned long durationMs) {
  // Start ramps for each servo
  baseRamp.start(currentBase, baseAngle, durationMs);
  shoulderRamp.start(currentShoulder, shoulderAngle, durationMs);
  elbowRamp.start(currentElbow, elbowAngle, durationMs);
  wristRamp.start(currentWrist, wristAngle, durationMs);
  
  unsigned long startTime = millis();
  
  // Update servos until all movements are complete
  while (!baseRamp.isDone() || !shoulderRamp.isDone() || !elbowRamp.isDone() || !wristRamp.isDone()) {
    // Update each servo position based on ramp
    float newBase = baseRamp.update();
    float newShoulder = shoulderRamp.update();
    float newElbow = elbowRamp.update();
    float newWrist = wristRamp.update();
    
    // Set servo positions
    setServo(BASE_SERVO, newBase);
    setServo(SHOULDER_SERVO, newShoulder);
    setServo(ELBOW_SERVO, newElbow);
    setServo(WRIST_SERVO, newWrist);
    
    // Update current positions
    currentBase = newBase;
    currentShoulder = newShoulder;
    currentElbow = newElbow;
    currentWrist = newWrist;
    
    delay(20); // Small delay to prevent overloading the I2C bus
  }
  
  Serial.println("Move completed in " + String(millis() - startTime) + "ms");
}

// Open the gripper
void openGripper() {
  setServo(GRIPPER_SERVO, GRIPPER_OPEN);
  Serial.println("Gripper opened");
  delay(500); // Allow time for gripper to open
}

// Close the gripper
void closeGripper() {
  setServo(GRIPPER_SERVO, GRIPPER_CLOSED);
  Serial.println("Gripper closed");
  delay(500); // Allow time for gripper to close
}

// Move to home position
void moveToHome() {
  Serial.println("Moving to home position");
  moveServosSmooth(HOME_BASE, HOME_SHOULDER, HOME_ELBOW, HOME_WRIST, 1500);
  openGripper();
}

// Inverse kinematics calculation - adjusted for -90 degree servo offset
// Calculate joint angles for a given end effector position
bool inverseKinematics(float x, float y, float z, float &baseAngle, float &shoulderAngle, float &elbowAngle, float &wristAngle) {
  // Calculate base angle (rotation around the z-axis)
  baseAngle = atan2(y, x) * 180.0 / PI;
  // Adjusted base angle for -90 degree offset
  baseAngle -= 90.0;
  
  // Adjust x and y to get the distance in the plane of the arm
  float r = sqrt(x*x + y*y);
  
  // Calculate the position for the wrist
  float wx = r;
  float wz = z;
  
  // Arm length (distance from shoulder to wrist)
  float arm_length = sqrt(wx*wx + wz*wz);
  
  // Check if the position is reachable
  if (arm_length > (SHOULDER_LENGTH + ELBOW_LENGTH)) {
    Serial.println("Position out of reach");
    return false;
  }
  
  // Law of cosines to find elbow angle
  float cos_elbow = (wx*wx + wz*wz - SHOULDER_LENGTH*SHOULDER_LENGTH - ELBOW_LENGTH*ELBOW_LENGTH) / 
                   (2 * SHOULDER_LENGTH * ELBOW_LENGTH);
                   
  // Constrain to prevent NaN due to floating point errors
  if (cos_elbow < -1.0) cos_elbow = -1.0;
  if (cos_elbow > 1.0) cos_elbow = 1.0;
  
  // Elbow angle (always positive in this configuration)
  elbowAngle = acos(cos_elbow) * 180.0 / PI;
  // In servo coordinate system (adjust as needed) and apply -90 offset
  elbowAngle = 90.0 - elbowAngle;  // Adjusted from 180-elbowAngle
  
  // Calculate shoulder angle
  float phi = atan2(wz, wx);
  float psi = atan2(ELBOW_LENGTH * sin(acos(cos_elbow)), 
                   SHOULDER_LENGTH + ELBOW_LENGTH * cos_elbow);
  shoulderAngle = (phi - psi) * 180.0 / PI;
  
  // Adjust shoulder angle for servo coordinate system and -90 offset
  shoulderAngle = 0.0 - shoulderAngle;  // Adjusted from 90-shoulderAngle
  
  // Set wrist angle to keep end effector level and apply -90 offset
  wristAngle = 0.0 - shoulderAngle - elbowAngle;  // Adjusted from 90-shoulderAngle-(180-elbowAngle)
  
  // Constrain angles to valid servo ranges (-90 to +90)
  if (baseAngle > 90) baseAngle = 90;
  if (baseAngle < -90) baseAngle = -90;
  
  if (shoulderAngle > 90) shoulderAngle = 90;
  if (shoulderAngle < -90) shoulderAngle = -90;
  
  if (elbowAngle > 90) elbowAngle = 90;
  if (elbowAngle < -90) elbowAngle = -90;
  
  if (wristAngle > 90) wristAngle = 90;
  if (wristAngle < -90) wristAngle = -90;
  
  return true;
}

// Move to a position in 3D space
bool moveToPosition(float x, float y, float z, unsigned long durationMs) {
  float baseAngle, shoulderAngle, elbowAngle, wristAngle;
  
  Serial.print("Moving to position (");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.print(z);
  Serial.println(")");
  
  if (inverseKinematics(x, y, z, baseAngle, shoulderAngle, elbowAngle, wristAngle)) {
    Serial.print("IK solution: Base=");
    Serial.print(baseAngle);
    Serial.print(", Shoulder=");
    Serial.print(shoulderAngle);
    Serial.print(", Elbow=");
    Serial.print(elbowAngle);
    Serial.print(", Wrist=");
    Serial.println(wristAngle);
    
    // Move servos smoothly to the calculated position
    moveServosSmooth(baseAngle, shoulderAngle, elbowAngle, wristAngle, durationMs);
    return true;
  } else {
    Serial.println("Failed to find IK solution");
    return false;
  }
}

// Pour water task
void pourWaterTask() {
  Serial.println("Starting water pouring task");
  
  // 1. Start at home position
  moveToHome();
  delay(1000);
  
  // 2. Move above beaker
  moveToPosition(BEAKER_X, BEAKER_Y, BEAKER_Z + 5.0, 1500);
  delay(500);
  
  // 3. Move down to beaker
  moveToPosition(BEAKER_X, BEAKER_Y, BEAKER_Z, 1000);
  delay(500);
  
  // 4. Close gripper to grab beaker
  closeGripper();
  delay(1000);
  
  // 5. Lift beaker up
  moveToPosition(BEAKER_X, BEAKER_Y, BEAKER_Z + 7.0, 1000);
  delay(500);
  
  // 6. Move to position above glass
  moveToPosition(GLASS_X, GLASS_Y, GLASS_Z + 10.0, 2000);
  delay(500);
  
  // 7. Pour water into glass (tilt wrist)
  Serial.println("Pouring water");
  float pourWrist = currentWrist - 45.0;  // Tilt 45 degrees to pour
  moveServosSmooth(currentBase, currentShoulder, currentElbow, pourWrist, 1500);
  delay(2000);  // Hold for pouring
  
  // 8. Return to upright position
  moveServosSmooth(currentBase, currentShoulder, currentElbow, currentWrist + 45.0, 1500);
  delay(500);
  
  // 9. Move back to position above beaker
  moveToPosition(BEAKER_X, BEAKER_Y, BEAKER_Z + 7.0, 2000);
  delay(500);
  
  // 10. Lower beaker to original position
  moveToPosition(BEAKER_X, BEAKER_Y, BEAKER_Z, 1000);
  delay(500);
  
  // 11. Release beaker
  openGripper();
  delay(1000);
  
  // 12. Move away from beaker
  moveToPosition(BEAKER_X, BEAKER_Y, BEAKER_Z + 5.0, 1000);
  delay(500);
  
  // 13. Return to home position
  moveToHome();
  
  Serial.println("Water pouring task completed!");
}
