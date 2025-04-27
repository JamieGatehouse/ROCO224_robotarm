#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// Create servo driver instance
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define servo channels
#define BASE_SERVO      0
#define SHOULDER_SERVO  1
#define ELBOW_SERVO     2
#define WRIST_PITCH     3
#define WRIST_ROLL      4
#define GRIPPER         5

// Define servo pulse ranges (microseconds)
#define SERVO_MIN_PULSE   600
#define SERVO_MAX_PULSE   2400
#define SERVO_FREQ        50  // 50 Hz update frequency

// Arm dimensions (cm)
#define HEIGHT_BASE_TO_SHOULDER  10
#define LENGTH_SHOULDER_TO_ELBOW 15
#define LENGTH_ELBOW_TO_WRIST    15
#define LENGTH_WRIST_TO_EFFECTOR 20

// Servo positions (degrees)
#define HOME_POSITION    90
#define GRIPPER_OPEN     0
#define GRIPPER_CLOSED   70

// Movement parameters
#define STEP_DELAY       15    // Delay between steps (ms)
#define ANGLE_STEP       1     // Step size for smooth movement (degrees)
#define POUR_STEP        2     // Smaller step for pouring (degrees)

// Current positions
int currentBase = HOME_POSITION;
int currentShoulder = HOME_POSITION;
int currentElbow = HOME_POSITION;
int currentWristPitch = HOME_POSITION;
int currentWristRoll = HOME_POSITION;
int currentGripper = HOME_POSITION;

void setup() {
  Serial.begin(115200);
  Serial.println("6-DOF Robot Arm Controller");
  
  // Initialize PWM driver
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  
  delay(1000);  // Allow time for servos to initialize
  
  // Move to home position
  moveToHome();
  delay(2000);
  
  // Execute the beaker task sequence
  executeBeakerTask();
}

void loop() {
  // Main program execution is done in setup
  // This allows the program to run once
}

// Convert degrees to PWM pulsewidth
int degreesToPulse(int degrees) {
  // Map degrees (0-180) to pulse width (600-2400)
  return map(degrees, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
}

// Set servo position directly
void setServoPosition(uint8_t servoNum, int degrees) {
  int pulse = degreesToPulse(degrees);
  pwm.writeMicroseconds(servoNum, pulse);
}

// Move servo with smooth transition
void moveServoSmooth(uint8_t servoNum, int *currentPos, int targetPos, int stepDelay) {
  // Determine direction
  int stepDir = (*currentPos < targetPos) ? ANGLE_STEP : -ANGLE_STEP;
  
  // Move in small steps
  while (abs(*currentPos - targetPos) > ANGLE_STEP) {
    *currentPos += stepDir;
    setServoPosition(servoNum, *currentPos);
    delay(stepDelay);
  }
  
  // Set final position
  *currentPos = targetPos;
  setServoPosition(servoNum, *currentPos);
}

// Move the arm to home position
void moveToHome() {
  // Move servos to home position simultaneously
  setServoPosition(BASE_SERVO, HOME_POSITION);
  setServoPosition(SHOULDER_SERVO, HOME_POSITION);
  setServoPosition(ELBOW_SERVO, HOME_POSITION);
  setServoPosition(WRIST_PITCH, HOME_POSITION);
  setServoPosition(WRIST_ROLL, HOME_POSITION);
  setServoPosition(GRIPPER, HOME_POSITION);
  
  // Update current positions
  currentBase = HOME_POSITION;
  currentShoulder = HOME_POSITION;
  currentElbow = HOME_POSITION;
  currentWristPitch = HOME_POSITION;
  currentWristRoll = HOME_POSITION;
  currentGripper = HOME_POSITION;
  
  Serial.println("Moved to home position");
}

// Inverse kinematics calculation
bool inverseKinematics(float x, float y, float z, int *baseAngle, int *shoulderAngle, int *elbowAngle, int *wristPitchAngle) {
  // Calculate base angle (rotation around y-axis)
  *baseAngle = round(atan2(y, x) * 180 / PI);
  
  // Convert to reach distance in the arm plane
  float r = sqrt(x*x + y*y);
  
  // Adjust for the wrist to reach the target point
  r -= LENGTH_WRIST_TO_EFFECTOR;
  
  // Calculate the height from the shoulder
  float adjustedZ = z - HEIGHT_BASE_TO_SHOULDER;
  
  // Calculate distance from shoulder to wrist
  float shoulderToWrist = sqrt(r*r + adjustedZ*adjustedZ);
  
  // Check if the point is reachable
  if (shoulderToWrist > (LENGTH_SHOULDER_TO_ELBOW + LENGTH_ELBOW_TO_WRIST) || 
      shoulderToWrist < abs(LENGTH_SHOULDER_TO_ELBOW - LENGTH_ELBOW_TO_WRIST)) {
    Serial.println("Position not reachable!");
    return false;
  }
  
  // Calculate angles using law of cosines
  float cosElbow = (pow(LENGTH_SHOULDER_TO_ELBOW, 2) + pow(LENGTH_ELBOW_TO_WRIST, 2) - pow(shoulderToWrist, 2)) / 
                   (2 * LENGTH_SHOULDER_TO_ELBOW * LENGTH_ELBOW_TO_WRIST);
  
  // Elbow angle - invert because servo is mounted in opposite direction
  *elbowAngle = 180 - round(acos(cosElbow) * 180 / PI);
  
  // Calculate shoulder angle
  float gamma = atan2(adjustedZ, r);
  float alpha = acos((pow(LENGTH_SHOULDER_TO_ELBOW, 2) + pow(shoulderToWrist, 2) - pow(LENGTH_ELBOW_TO_WRIST, 2)) / 
                     (2 * LENGTH_SHOULDER_TO_ELBOW * shoulderToWrist));
  *shoulderAngle = round((gamma + alpha) * 180 / PI);
  
  // Calculate wrist pitch to keep end effector horizontal
  *wristPitchAngle = 180 - *shoulderAngle - (180 - *elbowAngle);
  
  return true;
}

// Move the arm to a specific position
void moveArmToPosition(float x, float y, float z) {
  int baseAngle, shoulderAngle, elbowAngle, wristPitchAngle;
  
  Serial.print("Moving to position (");
  Serial.print(x); Serial.print(", ");
  Serial.print(y); Serial.print(", ");
  Serial.print(z); Serial.println(")");
  
  if (inverseKinematics(x, y, z, &baseAngle, &shoulderAngle, &elbowAngle, &wristPitchAngle)) {
    // Move base
    moveServoSmooth(BASE_SERVO, &currentBase, baseAngle, STEP_DELAY);
    
    // Move shoulder and elbow simultaneously
    int shoulderSteps = abs(shoulderAngle - currentShoulder);
    int elbowSteps = abs(elbowAngle - currentElbow);
    int maxSteps = max(shoulderSteps, elbowSteps);
    
    if (maxSteps > 0) {
      float shoulderStep = (shoulderAngle - currentShoulder) / (float)maxSteps;
      float elbowStep = (elbowAngle - currentElbow) / (float)maxSteps;
      
      float currentShoulderPos = currentShoulder;
      float currentElbowPos = currentElbow;
      
      for (int i = 0; i < maxSteps; i++) {
        currentShoulderPos += shoulderStep;
        currentElbowPos += elbowStep;
        
        setServoPosition(SHOULDER_SERVO, round(currentShoulderPos));
        setServoPosition(ELBOW_SERVO, round(currentElbowPos));
        delay(STEP_DELAY);
      }
    }
    
    // Set final positions
    setServoPosition(SHOULDER_SERVO, shoulderAngle);
    setServoPosition(ELBOW_SERVO, elbowAngle);
    
    // Update current positions
    currentShoulder = shoulderAngle;
    currentElbow = elbowAngle;
    
    // Move wrist pitch
    moveServoSmooth(WRIST_PITCH, &currentWristPitch, wristPitchAngle, STEP_DELAY);
    
    Serial.println("Position reached");
  } else {
    Serial.println("Failed to calculate inverse kinematics");
  }
}

// Execute the beaker task sequence
void executeBeakerTask() {
  // Open gripper
  Serial.println("Opening gripper");
  moveServoSmooth(GRIPPER, &currentGripper, GRIPPER_OPEN, STEP_DELAY);
  delay(500);
  
  // Move to beaker position (40cm in front, 10cm to the left)
  moveArmToPosition(40, -10, 0);
  delay(500);
  
  // Close gripper to grab beaker
  Serial.println("Grabbing beaker");
  moveServoSmooth(GRIPPER, &currentGripper, GRIPPER_CLOSED, STEP_DELAY);
  delay(1000);
  
  // Lift beaker 20cm off ground
  moveArmToPosition(40, -10, 20);
  delay(500);
  
  // Move to destination (40cm in front, 10cm to the right)
  moveArmToPosition(40, 10, 20);
  delay(500);
  
  // Lower to 10cm above ground
  moveArmToPosition(40, 10, 10);
  delay(1000);
  
  // Pour beaker (clockwise rotation)
  Serial.println("Pouring beaker");
  for (int angle = currentWristRoll; angle <= currentWristRoll + 90; angle += POUR_STEP) {
    setServoPosition(WRIST_ROLL, angle);
    delay(STEP_DELAY * 2);  // Slower pour
    currentWristRoll = angle;
  }
  
  // Hold for 3 seconds
  delay(3000);
  
  // Return beaker to upright position
  Serial.println("Returning beaker to upright position");
  for (int angle = currentWristRoll; angle >= currentWristRoll - 90; angle -= POUR_STEP) {
    setServoPosition(WRIST_ROLL, angle);
    delay(STEP_DELAY * 2);  // Slower rotation
    currentWristRoll = angle;
  }
  delay(500);
  
  // Return to original position
  moveArmToPosition(40, -10, 20);
  delay(500);
  
  // Lower to ground
  moveArmToPosition(40, -10, 0);
  delay(500);
  
  // Release beaker
  Serial.println("Releasing beaker");
  moveServoSmooth(GRIPPER, &currentGripper, GRIPPER_OPEN, STEP_DELAY);
  delay(500);
  
  // Return to home position
  Serial.println("Returning to home position");
  moveToHome();
  
  // Wait 10 seconds before next operation
  Serial.println("Task complete, waiting 10 seconds");
  delay(10000);
}