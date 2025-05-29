/*
Box-Moving Robot - ESP32 Arm Controller
Controls uArm servos, gripper, and manipulation operations
UART Communication with Raspberry Pi
*/

#include <ESP32Servo.h>
#include <HardwareSerial.h>

// Servo objects for uArm
Servo baseServo;      // Base rotation (0-180°)
Servo shoulderServo;  // Shoulder joint (0-180°)
Servo elbowServo;     // Elbow joint (0-180°)
Servo wristServo;     // Wrist rotation (0-180°)
Servo gripperServo;   // Gripper open/close (0-180°)

// Servo pin assignments
#define BASE_SERVO_PIN      12
#define SHOULDER_SERVO_PIN  13
#define ELBOW_SERVO_PIN     14
#define WRIST_SERVO_PIN     27
#define GRIPPER_SERVO_PIN   26

// Limit switches for gripper feedback
#define GRIP_SWITCH_1       34  // Left gripper contact
#define GRIP_SWITCH_2       35  // Right gripper contact

// Load cell for weight detection (optional)
#define LOAD_CELL_DOUT      32
#define LOAD_CELL_SCK       33

// LED indicators
#define STATUS_LED          2
#define ERROR_LED           4

// uArm physical constraints (adjust for your specific uArm model)
#define ARM_BASE_HEIGHT     120  // mm from ground to base servo
#define UPPER_ARM_LENGTH    148  // mm
#define FOREARM_LENGTH      160  // mm
#define GRIPPER_LENGTH      60   // mm

// Servo angle limits for safety
#define BASE_MIN_ANGLE      0
#define BASE_MAX_ANGLE      180
#define SHOULDER_MIN_ANGLE  0
#define SHOULDER_MAX_ANGLE  180
#define ELBOW_MIN_ANGLE     0
#define ELBOW_MAX_ANGLE     180
#define WRIST_MIN_ANGLE     0
#define WRIST_MAX_ANGLE     180
#define GRIPPER_OPEN_ANGLE  10
#define GRIPPER_CLOSE_ANGLE 90

// Servo positions
struct ServoPosition {
  int base;
  int shoulder;
  int elbow;
  int wrist;
  int gripper;
};

// Predefined positions
ServoPosition HOME_POSITION = {90, 45, 45, 90, GRIPPER_OPEN_ANGLE};
ServoPosition TRANSPORT_POSITION = {90, 90, 90, 90, GRIPPER_CLOSE_ANGLE};
ServoPosition PICKUP_READY = {90, 30, 120, 45, GRIPPER_OPEN_ANGLE};

// Current servo positions
ServoPosition currentPosition;

// Command processing
String incoming_command = "";
bool command_ready = false;
bool arm_busy = false;
bool object_gripped = false;

// Movement parameters
int movement_speed = 50;  // Delay between servo steps (ms)
int servo_step = 2;       // Degrees per step

void setup() {
  Serial.begin(115200);
  
  // Attach servos to pins
  baseServo.attach(BASE_SERVO_PIN);
  shoulderServo.attach(SHOULDER_SERVO_PIN);
  elbowServo.attach(ELBOW_SERVO_PIN);
  wristServo.attach(WRIST_SERVO_PIN);
  gripperServo.attach(GRIPPER_SERVO_PIN);
  
  // Initialize limit switches as inputs
  pinMode(GRIP_SWITCH_1, INPUT_PULLUP);
  pinMode(GRIP_SWITCH_2, INPUT_PULLUP);
  
  // Initialize LEDs
  pinMode(STATUS_LED, OUTPUT);
  pinMode(ERROR_LED, OUTPUT);
  
  // Move to home position
  moveToPosition(HOME_POSITION);
  currentPosition = HOME_POSITION;
  
  digitalWrite(STATUS_LED, HIGH);
  Serial.println("ARM:READY");
}

void loop() {
  // Check for incoming commands
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      command_ready = true;
    } else {
      incoming_command += c;
    }
  }
  
  // Process command if ready
  if (command_ready && !arm_busy) {
    processCommand(incoming_command);
    incoming_command = "";
    command_ready = false;
  }
  
  // Monitor gripper status
  checkGripperStatus();
  
  delay(10);
}

void processCommand(String command) {
  // Parse command format: ARM:ACTION:PARAM1:PARAM2
  int firstColon = command.indexOf(':');
  int secondColon = command.indexOf(':', firstColon + 1);
  int thirdColon = command.indexOf(':', secondColon + 1);
  
  if (firstColon == -1) return;
  
  String prefix = command.substring(0, firstColon);
  String action = command.substring(firstColon + 1, secondColon);
  String param1 = "";
  String param2 = "";
  
  if (secondColon != -1) {
    if (thirdColon != -1) {
      param1 = command.substring(secondColon + 1, thirdColon);
      param2 = command.substring(thirdColon + 1);
    } else {
      param1 = command.substring(secondColon + 1);
    }
  }
  
  // Process arm commands
  if (prefix == "ARM") {
    arm_busy = true;
    digitalWrite(STATUS_LED, LOW); // Indicate busy
    
    if (action == "HOME") {
      handleHome();
    }
    else if (action == "PICK") {
      handlePick(param1.toInt(), param2.toInt());
    }
    else if (action == "PLACE") {
      handlePlace(param1.toInt(), param2.toInt());
    }
    else if (action == "TRANSPORT") {
      handleTransport();
    }
    else if (action == "GRIP") {
      handleGrip(param1);
    }
    else if (action == "MOVE") {
      handleMove(param1.toInt(), param2.toInt());
    }
    else if (action == "STOP") {
      handleStop();
    }
    else if (action == "STATUS") {
      handleStatus();
    }
    else {
      Serial.println("ARM:ERROR:UNKNOWN_COMMAND");
    }
    
    arm_busy = false;
    digitalWrite(STATUS_LED, HIGH); // Indicate ready
  }
}

void handleHome() {
  Serial.println("ARM:MOVING_HOME");
  
  if (moveToPosition(HOME_POSITION)) {
    currentPosition = HOME_POSITION;
    object_gripped = false;
    Serial.println("ARM:SUCCESS:HOME_COMPLETE");
  } else {
    Serial.println("ARM:ERROR:HOME_FAILED");
  }
}

void handlePick(int x, int y) {
  Serial.println("ARM:EXECUTING_PICK:X" + String(x) + "_Y" + String(y));
  
  // Calculate inverse kinematics for pickup position
  ServoPosition pickupPos = calculateInverseKinematics(x, y, 0); // Z=0 for ground level
  
  if (!isPositionValid(pickupPos)) {
    Serial.println("ARM:ERROR:POSITION_OUT_OF_REACH");
    return;
  }
  
  // Execute pickup sequence
  if (executePickupSequence(pickupPos)) {
    Serial.println("ARM:SUCCESS:PICK_COMPLETE");
  } else {
    Serial.println("ARM:ERROR:PICK_FAILED");
  }
}

void handlePlace(int x, int y) {
  Serial.println("ARM:EXECUTING_PLACE:X" + String(x) + "_Y" + String(y));
  
  if (!object_gripped) {
    Serial.println("ARM:ERROR:NO_OBJECT_TO_PLACE");
    return;
  }
  
  // Calculate position for placement (slightly higher than pickup)
  ServoPosition placePos = calculateInverseKinematics(x, y, 50); // Z=50mm above ground
  
  if (!isPositionValid(placePos)) {
    Serial.println("ARM:ERROR:POSITION_OUT_OF_REACH");
    return;
  }
  
  // Execute placement sequence
  if (executePlacementSequence(placePos)) {
    Serial.println("ARM:SUCCESS:PLACE_COMPLETE");
  } else {
    Serial.println("ARM:ERROR:PLACE_FAILED");
  }
}

void handleTransport() {
  Serial.println("ARM:MOVING_TO_TRANSPORT");
  
  if (moveToPosition(TRANSPORT_POSITION)) {
    currentPosition = TRANSPORT_POSITION;
    Serial.println("ARM:SUCCESS:TRANSPORT_POSITION");
  } else {
    Serial.println("ARM:ERROR:TRANSPORT_FAILED");
  }
}

void handleGrip(String action) {
  if (action == "OPEN") {
    Serial.println("ARM:OPENING_GRIPPER");
    if (openGripper()) {
      object_gripped = false;
      Serial.println("ARM:SUCCESS:GRIPPER_OPENED");
    } else {
      Serial.println("ARM:ERROR:GRIPPER_OPEN_FAILED");
    }
  }
  else if (action == "CLOSE") {
    Serial.println("ARM:CLOSING_GRIPPER");
    if (closeGripper()) {
      object_gripped = checkGripperContact();
      if (object_gripped) {
        Serial.println("ARM:SUCCESS:OBJECT_GRIPPED");
      } else {
        Serial.println("ARM:WARNING:NO_OBJECT_DETECTED");
      }
    } else {
      Serial.println("ARM:ERROR:GRIPPER_CLOSE_FAILED");
    }
  }
  else if (action == "STATUS") {
    bool contact = checkGripperContact();
    if (contact) {
      Serial.println("ARM:GRIP_STATUS:GRIPPED");
    } else {
      Serial.println("ARM:GRIP_STATUS:EMPTY");
    }
  }
}

void handleMove(int x, int y) {
  Serial.println("ARM:MOVING_TO:X" + String(x) + "_Y" + String(y));
  
  ServoPosition targetPos = calculateInverseKinematics(x, y, 100); // 100mm height
  
  if (!isPositionValid(targetPos)) {
    Serial.println("ARM:ERROR:POSITION_OUT_OF_REACH");
    return;
  }
  
  if (moveToPosition(targetPos)) {
    currentPosition = targetPos;
    Serial.println("ARM:SUCCESS:MOVE_COMPLETE");
  } else {
    Serial.println("ARM:ERROR:MOVE_FAILED");
  }
}

void handleStop() {
  // Emergency stop - freeze all servos in current position
  arm_busy = false;
  digitalWrite(ERROR_LED, HIGH);
  Serial.println("ARM:SUCCESS:EMERGENCY_STOP");
}

void handleStatus() {
  String status = "ARM:STATUS:";
  status += "BASE:" + String(currentPosition.base) + ",";
  status += "SHOULDER:" + String(currentPosition.shoulder) + ",";
  status += "ELBOW:" + String(currentPosition.elbow) + ",";
  status += "WRIST:" + String(currentPosition.wrist) + ",";
  status += "GRIPPER:" + String(currentPosition.gripper) + ",";
  status += "GRIPPED:" + String(object_gripped ? "TRUE" : "FALSE");
  
  Serial.println(status);
}

bool executePickupSequence(ServoPosition pickupPos) {
  // 1. Move to approach position (above target)
  ServoPosition approachPos = pickupPos;
  approachPos.elbow -= 20; // Lift arm higher
  
  if (!moveToPosition(approachPos)) {
    return false;
  }
  
  // 2. Open gripper
  if (!openGripper()) {
    return false;
  }
  
  // 3. Lower to pickup position
  if (!moveToPosition(pickupPos)) {
    return false;
  }
  
  // 4. Close gripper
  if (!closeGripper()) {
    return false;
  }
  
  // 5. Check if object is gripped
  delay(500); // Allow time for grip to settle
  object_gripped = checkGripperContact();
  
  if (!object_gripped) {
    Serial.println("ARM:WARNING:PICKUP_UNCERTAIN");
  }
  
  // 6. Lift object
  if (!moveToPosition(approachPos)) {
    return false;
  }
  
  return true;
}

bool executePlacementSequence(ServoPosition placePos) {
  // 1. Move to approach position (above target)
  ServoPosition approachPos = placePos;
  approachPos.elbow -= 15; // Slightly higher
  
  if (!moveToPosition(approachPos)) {
    return false;
  }
  
  // 2. Lower to placement position
  if (!moveToPosition(placePos)) {
    return false;
  }
  
  // 3. Open gripper to release object
  if (!openGripper()) {
    return false;
  }
  
  object_gripped = false;
  
  // 4. Move back to approach position
  if (!moveToPosition(approachPos)) {
    return false;
  }
  
  return true;
}

bool moveToPosition(ServoPosition targetPos) {
  // Smooth movement by interpolating between current and target positions
  int steps = calculateMaxSteps(currentPosition, targetPos);
  
  for (int step = 0; step <= steps; step++) {
    ServoPosition intermediatePos;
    
    // Linear interpolation for each servo
    intermediatePos.base = map(step, 0, steps, currentPosition.base, targetPos.base);
    intermediatePos.shoulder = map(step, 0, steps, currentPosition.shoulder, targetPos.shoulder);
    intermediatePos.elbow = map(step, 0, steps, currentPosition.elbow, targetPos.elbow);
    intermediatePos.wrist = map(step, 0, steps, currentPosition.wrist, targetPos.wrist);
    intermediatePos.gripper = map(step, 0, steps, currentPosition.gripper, targetPos.gripper);
    
    // Apply position to servos
    setServoPositions(intermediatePos);
    
    delay(movement_speed);
    
    // Check for emergency stop
    if (!arm_busy) {
      return false;
    }
  }
  
  currentPosition = targetPos;
  return true;
}

void setServoPositions(ServoPosition pos) {
  // Apply safety limits
  pos.base = constrain(pos.base, BASE_MIN_ANGLE, BASE_MAX_ANGLE);
  pos.shoulder = constrain(pos.shoulder, SHOULDER_MIN_ANGLE, SHOULDER_MAX_ANGLE);
  pos.elbow = constrain(pos.elbow, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE);
  pos.wrist = constrain(pos.wrist, WRIST_MIN_ANGLE, WRIST_MAX_ANGLE);
  pos.gripper = constrain(pos.gripper, GRIPPER_OPEN_ANGLE, GRIPPER_CLOSE_ANGLE);
  
  // Write to servos
  baseServo.write(pos.base);
  shoulderServo.write(pos.shoulder);
  elbowServo.write(pos.elbow);
  wristServo.write(pos.wrist);
  gripperServo.write(pos.gripper);
}

int calculateMaxSteps(ServoPosition current, ServoPosition target) {
  int maxDiff = 0;
  maxDiff = max(maxDiff, abs(target.base - current.base));
  maxDiff = max(maxDiff, abs(target.shoulder - current.shoulder));
  maxDiff = max(maxDiff, abs(target.elbow - current.elbow));
  maxDiff = max(maxDiff, abs(target.wrist - current.wrist));
  maxDiff = max(maxDiff, abs(target.gripper - current.gripper));
  
  return maxDiff / servo_step;
}

ServoPosition calculateInverseKinematics(int x, int y, int z) {
  // Simplified inverse kinematics for uArm
  // This is a basic implementation - adjust for your specific uArm model
  
  ServoPosition result;
  
  // Calculate base rotation
  result.base = map(atan2(y, x) * 180 / PI, -90, 90, 0, 180);
  
  // Calculate reach distance
  float reach = sqrt(x*x + y*y);
  float height = z + ARM_BASE_HEIGHT;
  
  // Calculate shoulder and elbow angles using geometry
  float r = sqrt(reach*reach + height*height);
  
  if (r > (UPPER_ARM_LENGTH + FOREARM_LENGTH)) {
    // Target is out of reach, return maximum extension
    result.shoulder = 0;
    result.elbow = 0;
  } else {
    // Calculate using law of cosines
    float angle1 = acos((UPPER_ARM_LENGTH*UPPER_ARM_LENGTH + r*r - FOREARM_LENGTH*FOREARM_LENGTH) / (2*UPPER_ARM_LENGTH*r));
    float angle2 = atan2(height, reach);
    
    result.shoulder = (angle1 + angle2) * 180 / PI;
    
    float angle3 = acos((UPPER_ARM_LENGTH*UPPER_ARM_LENGTH + FOREARM_LENGTH*FOREARM_LENGTH - r*r) / (2*UPPER_ARM_LENGTH*FOREARM_LENGTH));
    result.elbow = (PI - angle3) * 180 / PI;
  }
  
  // Set wrist to keep gripper horizontal
  result.wrist = 180 - result.shoulder - result.elbow;
  
  // Keep current gripper position
  result.gripper = currentPosition.gripper;
  
  return result;
}

bool isPositionValid(ServoPosition pos) {
  return (pos.base >= BASE_MIN_ANGLE && pos.base <= BASE_MAX_ANGLE &&
          pos.shoulder >= SHOULDER_MIN_ANGLE && pos.shoulder <= SHOULDER_MAX_ANGLE &&
          pos.elbow >= ELBOW_MIN_ANGLE && pos.elbow <= ELBOW_MAX_ANGLE &&
          pos.wrist >= WRIST_MIN_ANGLE && pos.wrist <= WRIST_MAX_ANGLE);
}

bool openGripper() {
  ServoPosition openPos = currentPosition;
  openPos.gripper = GRIPPER_OPEN_ANGLE;
  
  return moveToPosition(openPos);
}

bool closeGripper() {
  ServoPosition closePos = currentPosition;
  closePos.gripper = GRIPPER_CLOSE_ANGLE;
  
  // Close gradually and check for contact
  for (int angle = currentPosition.gripper; angle <= GRIPPER_CLOSE_ANGLE; angle += 5) {
    gripperServo.write(angle);
    delay(100);
    
    if (checkGripperContact()) {
      currentPosition.gripper = angle;
      return true;
    }
  }
  
  currentPosition.gripper = GRIPPER_CLOSE_ANGLE;
  return true;
}

bool checkGripperContact() {
  // Check limit switches to detect object contact
  bool switch1 = !digitalRead(GRIP_SWITCH_1); // Inverted because of pull-up
  bool switch2 = !digitalRead(GRIP_SWITCH_2);
  
  return (switch1 || switch2);
}

void checkGripperStatus() {
  static unsigned long lastCheck = 0;
  
  if (millis() - lastCheck > 1000) { // Check every second
    if (object_gripped && !checkGripperContact()) {
      Serial.println("ARM:WARNING:OBJECT_LOST");
      object_gripped = false;
    }
    lastCheck = millis();
  }
}