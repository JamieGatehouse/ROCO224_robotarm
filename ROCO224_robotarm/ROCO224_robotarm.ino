#include <Wire.h>                       // Library for I2C communication
#include <Adafruit_PWMServoDriver.h>    // Library to control servos via PWM
#include <math.h>                       // For mathematical operations (sin, cos, etc.)

// Create servo driver instance - uses default I2C address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define servo channels (which port on the PWM driver each servo is connected to)
#define BASE_SERVO      0    // Controls rotation around the base (horizontal plane)
#define SHOULDER_SERVO  1    // Controls the shoulder joint (up/down movement)
#define ELBOW_SERVO     2    // Controls the elbow joint (up/down movement)
#define WRIST_PITCH     3    // Controls wrist up/down movement
#define WRIST_ROLL      4    // Controls wrist rotation (for pouring)
#define GRIPPER         5    // Controls the gripper (open/close)

// Define servo pulse ranges (in microseconds)
// These values determine the min/max positions of the servo
#define SERVO_MIN_PULSE   600    // Minimum pulse width for 0 degrees
#define SERVO_MAX_PULSE   2400   // Maximum pulse width for 180 degrees
#define SERVO_FREQ        50     // 50 Hz update frequency (standard for most servos)

// Physical dimensions of the robot arm (in cm)
#define HEIGHT_BASE_TO_SHOULDER  10    // Height from base to shoulder joint
#define LENGTH_SHOULDER_TO_ELBOW 15    // Length of upper arm segment
#define LENGTH_ELBOW_TO_WRIST    15    // Length of forearm segment
#define LENGTH_WRIST_TO_EFFECTOR 20    // Length from wrist to end of gripper

// Servo position definitions (in degrees)
#define HOME_POSITION    90     // Middle position for servos (90 degrees)
#define GRIPPER_OPEN     180    // Fully open gripper position
#define GRIPPER_CLOSED   0      // Fully closed gripper position (to grab objects)

// Movement parameters for smooth operation
#define STEP_DELAY       15     // Delay between steps (ms) - affects movement speed
#define ANGLE_STEP       1      // Step size for normal movements (degrees)
#define POUR_STEP        2      // Step size for pouring movement (smaller for precision)

// Variables to track current position of each servo (in degrees)
int currentBase = HOME_POSITION;
int currentShoulder = HOME_POSITION;
int currentElbow = HOME_POSITION;
int currentWristPitch = HOME_POSITION;
int currentWristRoll = HOME_POSITION;
int currentGripper = HOME_POSITION;

void setup() {
  // Initialize serial communication for debugging and status messages
  Serial.begin(9600);
  Serial.println("6-DOF Robot Arm Controller");
  
  // Initialize the PWM servo driver
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);  // Set the PWM update frequency
  
  delay(1000);  // Allow time for servos to initialize
  
  // Move all servos to home position
  moveToHome();
  delay(2000);  // Wait 2 seconds
  
  // Execute the main task sequence
  executeBeakerTask();
}

void loop() {
  // Main program execution is done in setup
  // This allows the program to run once and stop
  // The loop remains empty as we only want to execute the sequence once
}

// Convert angle in degrees to PWM pulse width in microseconds
int degreesToPulse(int degrees) {
  // Map the angle (0-180 degrees) to the corresponding pulse width
  return map(degrees, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
}

// Set servo position directly (no smooth movement)
void setServoPosition(uint8_t servoNum, int degrees) {
  int pulse = degreesToPulse(degrees);  // Convert degrees to pulse width
  pwm.writeMicroseconds(servoNum, pulse);  // Set the servo position
}

// Move servo with smooth transition from current position to target position
void moveServoSmooth(uint8_t servoNum, int *currentPos, int targetPos, int stepDelay) {
  // Determine direction of movement (positive or negative)
  int stepDir = (*currentPos < targetPos) ? ANGLE_STEP : -ANGLE_STEP;
  
  // Move in small steps to create smooth motion
  while (abs(*currentPos - targetPos) > ANGLE_STEP) {
    *currentPos += stepDir;  // Update position by one step
    setServoPosition(servoNum, *currentPos);  // Move servo
    delay(stepDelay);  // Wait before next step
  }
  
  // Set final position to ensure we reach exact target
  *currentPos = targetPos;
  setServoPosition(servoNum, *currentPos);
}

// Move the arm to home position (all servos at 90 degrees)
void moveToHome() {
  // Move all servos to home position simultaneously
  setServoPosition(BASE_SERVO, HOME_POSITION);
  setServoPosition(SHOULDER_SERVO, HOME_POSITION);
  setServoPosition(ELBOW_SERVO, HOME_POSITION);
  setServoPosition(WRIST_PITCH, HOME_POSITION);
  setServoPosition(WRIST_ROLL, HOME_POSITION);
  setServoPosition(GRIPPER, HOME_POSITION);
  
  // Update current position variables
  currentBase = HOME_POSITION;
  currentShoulder = HOME_POSITION;
  currentElbow = HOME_POSITION;
  currentWristPitch = HOME_POSITION;
  currentWristRoll = HOME_POSITION;
  currentGripper = HOME_POSITION;
  
  Serial.println("Moved to home position");
}

// Calculate inverse kinematics to determine servo angles for a given 3D position
bool inverseKinematics(float x, float y, float z, int *baseAngle, int *shoulderAngle, int *elbowAngle, int *wristPitchAngle) {
  // Calculate base angle (rotation around y-axis)
  // atan2 returns angle in radians, convert to degrees
  *baseAngle = round(atan2(y, x) * 180 / PI);
  
  // Limit base angle to prevent excessive movement
  if (*baseAngle > 45) *baseAngle = 45;  // Limit rightward rotation
  if (*baseAngle < -45) *baseAngle = -45;  // Limit leftward rotation
  
  // Map calculated angle to servo angle (90 is center position)
  *baseAngle = 90 + *baseAngle;
  
  // Calculate horizontal distance in arm plane (using Pythagorean theorem)
  float r = sqrt(x*x + y*y);
  
  // Adjust reach distance to account for wrist-to-effector length
  r -= LENGTH_WRIST_TO_EFFECTOR;
  
  // Calculate the height from the shoulder joint
  float adjustedZ = z - HEIGHT_BASE_TO_SHOULDER;
  
  // Calculate straight-line distance from shoulder to wrist
  float shoulderToWrist = sqrt(r*r + adjustedZ*adjustedZ);
  
  // Check if the target point is reachable
  // Point is unreachable if:
  // 1. It's beyond the maximum reach of the arm
  // 2. It's closer than the minimum reach (when arm is folded)
  if (shoulderToWrist > (LENGTH_SHOULDER_TO_ELBOW + LENGTH_ELBOW_TO_WRIST) || 
      shoulderToWrist < abs(LENGTH_SHOULDER_TO_ELBOW - LENGTH_ELBOW_TO_WRIST)) {
    Serial.println("Position not reachable!");
    return false;
  }
  
  // Calculate elbow angle using law of cosines
  // cos(C) = (a² + b² - c²) / (2ab) where:
  // C = elbow angle, a = upper arm length, b = forearm length, c = distance from shoulder to wrist
  float cosElbow = (pow(LENGTH_SHOULDER_TO_ELBOW, 2) + pow(LENGTH_ELBOW_TO_WRIST, 2) - pow(shoulderToWrist, 2)) / 
                   (2 * LENGTH_SHOULDER_TO_ELBOW * LENGTH_ELBOW_TO_WRIST);
  
  // Ensure value is within valid range to avoid NaN in acos function
  cosElbow = constrain(cosElbow, -1.0, 1.0);
  
  // Calculate elbow angle (invert because servo is mounted in opposite direction)
  *elbowAngle = 180 - round(acos(cosElbow) * 180 / PI);
  
  // Calculate shoulder angle (composed of two parts)
  float gamma = atan2(adjustedZ, r);  // Angle to the target from horizontal
  
  // Use law of cosines again to find the angle between upper arm and line to target
  float alpha = acos((pow(LENGTH_SHOULDER_TO_ELBOW, 2) + pow(shoulderToWrist, 2) - pow(LENGTH_ELBOW_TO_WRIST, 2)) / 
                     (2 * LENGTH_SHOULDER_TO_ELBOW * shoulderToWrist));
  
  // Shoulder angle is sum of these two angles
  *shoulderAngle = round((gamma + alpha) * 180 / PI);
  
  // Calculate wrist pitch to keep end effector horizontal
  // Total angle in the vertical plane must be 180 degrees (straight)
  *wristPitchAngle = 180 - *shoulderAngle - (180 - *elbowAngle);
  
  return true;  // Return success
}

// Move the arm to a specific XYZ position
void moveArmToPosition(float x, float y, float z) {
  int baseAngle, shoulderAngle, elbowAngle, wristPitchAngle;
  
  // Print target position for debugging
  Serial.print("Moving to position (");
  Serial.print(x); Serial.print(", ");
  Serial.print(y); Serial.print(", ");
  Serial.print(z); Serial.println(")");
  
  // Calculate required joint angles using inverse kinematics
  if (inverseKinematics(x, y, z, &baseAngle, &shoulderAngle, &elbowAngle, &wristPitchAngle)) {
    // Move base first to avoid arm collision with obstacles
    moveServoSmooth(BASE_SERVO, &currentBase, baseAngle, STEP_DELAY);
    delay(200);  // Short delay after base rotation
    
    // Move shoulder and elbow simultaneously for more natural movement
    int shoulderSteps = abs(shoulderAngle - currentShoulder);
    int elbowSteps = abs(elbowAngle - currentElbow);
    int maxSteps = max(shoulderSteps, elbowSteps);  // Find which joint moves more
    
    if (maxSteps > 0) {
      // Calculate step size for each joint to finish movement at same time
      float shoulderStep = (shoulderAngle - currentShoulder) / (float)maxSteps;
      float elbowStep = (elbowAngle - currentElbow) / (float)maxSteps;
      
      float currentShoulderPos = currentShoulder;
      float currentElbowPos = currentElbow;
      
      // Perform simultaneous movement
      for (int i = 0; i < maxSteps; i++) {
        currentShoulderPos += shoulderStep;
        currentElbowPos += elbowStep;
        
        // Set new positions
        setServoPosition(SHOULDER_SERVO, round(currentShoulderPos));
        setServoPosition(ELBOW_SERVO, round(currentElbowPos));
        delay(STEP_DELAY);
      }
    }
    
    // Set final positions exactly
    setServoPosition(SHOULDER_SERVO, shoulderAngle);
    setServoPosition(ELBOW_SERVO, elbowAngle);
    
    // Update current position variables
    currentShoulder = shoulderAngle;
    currentElbow = elbowAngle;
    
    // Move wrist pitch last
    moveServoSmooth(WRIST_PITCH, &currentWristPitch, wristPitchAngle, STEP_DELAY);
    
    Serial.println("Position reached");
  } else {
    Serial.println("Failed to calculate inverse kinematics");
  }
}

// Main function to execute the beaker task sequence
void executeBeakerTask() {
  // Step 1: Open gripper to prepare for picking up
  Serial.println("Opening gripper");
  moveServoSmooth(GRIPPER, &currentGripper, GRIPPER_OPEN, STEP_DELAY);
  delay(500);  // Wait for gripper to fully open
  
  // Step 2: Move to beaker position (40cm in front, 10cm to the left, 10cm below base)
  moveArmToPosition(40, -10, -10);
  delay(500);
  
  // Step 3: Adjust elbow and wrist for proper pickup angle
  Serial.println("Adjusting elbow and wrist pitch to 90 degrees");
  moveServoSmooth(ELBOW_SERVO, &currentElbow, 90, STEP_DELAY);
  delay(300);
  moveServoSmooth(WRIST_PITCH, &currentWristPitch, 90, STEP_DELAY);
  delay(500);
  
  // Step 4: Close gripper to grab beaker
  Serial.println("Grabbing beaker");
  moveServoSmooth(GRIPPER, &currentGripper, GRIPPER_CLOSED, STEP_DELAY);
  delay(1000);  // Ensure firm grip before moving
  
  // Step 5: Lift beaker 20cm above ground level
  moveArmToPosition(40, -10, 20);
  delay(500);
  
  // Step 6: Move to destination position (40cm in front, 10cm to the right)
  moveArmToPosition(40, 10, 20);
  delay(500);
  
  // Step 7: Lower to 10cm above ground for pouring
  moveArmToPosition(40, 10, 10);
  delay(1000);
  
  // Remember initial wrist roll position
  int initialWristRoll = currentWristRoll;
  
  // Step 8: Pour beaker by rotating wrist 90 degrees clockwise
  Serial.println("Pouring beaker");
  int targetAngle = initialWristRoll + 90;  // 90 degrees clockwise
  
  // Gradually rotate for a controlled pour
  for (int angle = initialWristRoll; angle <= targetAngle; angle += POUR_STEP) {
    setServoPosition(WRIST_ROLL, angle);
    currentWristRoll = angle;  // Update current position
    delay(STEP_DELAY * 2);     // Slower pour for better control
  }
  
  // Hold pouring position for 3 seconds
  delay(3000);
  
  // Step 9: Return beaker to upright position
  Serial.println("Returning beaker to upright position");
  
  // Gradually rotate back to upright
  for (int angle = currentWristRoll; angle >= initialWristRoll; angle -= POUR_STEP) {
    setServoPosition(WRIST_ROLL, angle);
    currentWristRoll = angle;  // Update current position
    delay(STEP_DELAY * 2);     // Slower rotation for better control
  }
  delay(500);
  
  // Step 10: Return to original position
  moveArmToPosition(40, -10, 20);
  delay(500);
  
  // Step 11: Lower back to ground level for placement
  moveArmToPosition(40, -10, -10);
  delay(500);
  
  // Step 12: Adjust elbow and wrist for proper placement
  Serial.println("Adjusting elbow and wrist pitch to 90 degrees");
  moveServoSmooth(ELBOW_SERVO, &currentElbow, 90, STEP_DELAY);
  delay(300);
  moveServoSmooth(WRIST_PITCH, &currentWristPitch, 90, STEP_DELAY);
  delay(500);
  
  // Step 13: Release beaker
  Serial.println("Releasing beaker");
  moveServoSmooth(GRIPPER, &currentGripper, GRIPPER_OPEN, STEP_DELAY);
  delay(500);
  
  // Step 14: Return to home position safely
  Serial.println("Returning to home position");
  
  // First lift arm slightly to avoid collision with objects
  moveServoSmooth(ELBOW_SERVO, &currentElbow, 110, STEP_DELAY);
  delay(300);
  
  // Move servos to home position one by one for smooth transition
  moveServoSmooth(WRIST_ROLL, &currentWristRoll, HOME_POSITION, STEP_DELAY);
  delay(200);
  moveServoSmooth(WRIST_PITCH, &currentWristPitch, HOME_POSITION, STEP_DELAY);
  delay(200);
  
  // Move shoulder and elbow together gradually for natural motion
  int shoulderSteps = abs(HOME_POSITION - currentShoulder);
  int elbowSteps = abs(HOME_POSITION - currentElbow);
  int maxSteps = max(shoulderSteps, elbowSteps);
  
  if (maxSteps > 0) {
    float shoulderStep = (HOME_POSITION - currentShoulder) / (float)maxSteps;
    float elbowStep = (HOME_POSITION - currentElbow) / (float)maxSteps;
    
    float currentShoulderPos = currentShoulder;
    float currentElbowPos = currentElbow;
    
    // Perform simultaneous movement
    for (int i = 0; i < maxSteps; i++) {
      currentShoulderPos += shoulderStep;
      currentElbowPos += elbowStep;
      
      setServoPosition(SHOULDER_SERVO, round(currentShoulderPos));
      setServoPosition(ELBOW_SERVO, round(currentElbowPos));
      delay(STEP_DELAY * 2);  // Slower movement for smoother transition
    }
  }
  
  // Set final positions and update current position variables
  setServoPosition(SHOULDER_SERVO, HOME_POSITION);
  setServoPosition(ELBOW_SERVO, HOME_POSITION);
  currentShoulder = HOME_POSITION;
  currentElbow = HOME_POSITION;
  
  // Finally move base back to center
  moveServoSmooth(BASE_SERVO, &currentBase, HOME_POSITION, STEP_DELAY);
  
  // Set gripper to home position
  moveServoSmooth(GRIPPER, &currentGripper, HOME_POSITION, STEP_DELAY);
  
  Serial.println("Home position reached");
  
  // Wait 10 seconds before next operation (if program were to continue)
  Serial.println("Task complete, waiting 10 seconds");
  delay(10000);
}