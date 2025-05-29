/*
Box-Moving Robot - ESP32 Navigation Controller
Handles motors, ultrasonic sensors, and movement commands
UART Communication with Raspberry Pi
*/

#include <HardwareSerial.h>
#include <ESP32Servo.h>

// Motor pins (L298N driver)
#define MOTOR_LEFT_PWM    12
#define MOTOR_LEFT_DIR1   14
#define MOTOR_LEFT_DIR2   27
#define MOTOR_RIGHT_PWM   13
#define MOTOR_RIGHT_DIR1  26
#define MOTOR_RIGHT_DIR2  25

// Ultrasonic sensor pins
#define TRIG_FRONT        5
#define ECHO_FRONT        18
#define TRIG_LEFT         19
#define ECHO_LEFT         21
#define TRIG_RIGHT        22
#define ECHO_RIGHT        23

// Encoder pins (optional)
#define ENCODER_LEFT_A    34
#define ENCODER_LEFT_B    35
#define ENCODER_RIGHT_A   32
#define ENCODER_RIGHT_B   33

// Robot parameters
#define WHEEL_DIAMETER    65    // mm
#define WHEEL_BASE        200   // mm (distance between wheels)
#define PULSES_PER_REV    20    // Encoder pulses per revolution
#define MAX_SPEED         255   // PWM max value

// Global variables
volatile long left_encoder_count = 0;
volatile long right_encoder_count = 0;
float robot_x = 0.0;
float robot_y = 0.0;
float robot_angle = 0.0;

String incoming_command = "";
bool command_ready = false;

// PID control variables
float kp = 2.0, ki = 0.1, kd = 0.05;
float left_error_sum = 0, right_error_sum = 0;
float left_last_error = 0, right_last_error = 0;

void setup() {
  Serial.begin(115200);
  
  // Initialize motor pins
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_DIR1, OUTPUT);
  pinMode(MOTOR_LEFT_DIR2, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR1, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR2, OUTPUT);
  
  // Initialize ultrasonic pins
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  
  // Initialize encoder pins
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
  
  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), rightEncoderISR, RISING);
  
  // Stop motors initially
  stopMotors();
  
  Serial.println("NAV:READY");
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
  if (command_ready) {
    processCommand(incoming_command);
    incoming_command = "";
    command_ready = false;
  }
  
  // Small delay to prevent overwhelming the system
  delay(10);
}

void processCommand(String command) {
  // Parse command format: NAV:ACTION:PARAM1:PARAM2
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
  
  // Process navigation commands
  if (prefix == "NAV") {
    if (action == "MOVE") {
      handleMovement(param1, param2.toInt());
    }
    else if (action == "TURN") {
      handleTurning(param1, param2.toInt());
    }
    else if (action == "STOP") {
      handleStop();
    }
    else if (action == "SCAN") {
      handleSensorScan();
    }
    else if (action == "POSITION") {
      handlePositioning(param1);
    }
    else if (action == "RESET") {
      handleReset();
    }
    else {
      Serial.println("NAV:ERROR:UNKNOWN_COMMAND");
    }
  }
}

void handleMovement(String direction, int distance) {
  Serial.println("NAV:MOVING:" + direction + ":" + String(distance));
  
  if (direction == "FORWARD") {
    moveForward(distance);
  }
  else if (direction == "BACKWARD") {
    moveBackward(distance);
  }
  else if (direction == "LEFT") {
    strafeLeft(distance);
  }
  else if (direction == "RIGHT") {
    strafeRight(distance);
  }
  else {
    Serial.println("NAV:ERROR:INVALID_DIRECTION");
    return;
  }
  
  Serial.println("NAV:SUCCESS:MOVE_COMPLETE");
}

void handleTurning(String direction, int angle) {
  Serial.println("NAV:TURNING:" + direction + ":" + String(angle));
  
  if (direction == "LEFT") {
    turnLeft(angle);
  }
  else if (direction == "RIGHT") {
    turnRight(angle);
  }
  else {
    Serial.println("NAV:ERROR:INVALID_TURN_DIRECTION");
    return;
  }
  
  Serial.println("NAV:SUCCESS:TURN_COMPLETE");
}

void handleStop() {
  stopMotors();
  Serial.println("NAV:SUCCESS:STOPPED");
}

void handleSensorScan() {
  float front_dist = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  float left_dist = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
  float right_dist = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);
  
  String response = "NAV:SENSORS:FRONT:" + String(front_dist) + 
                   ",LEFT:" + String(left_dist) + 
                   ",RIGHT:" + String(right_dist);
  
  // Check for obstacles
  if (front_dist < 20 || left_dist < 15 || right_dist < 15) {
    response += ":OBSTACLE_DETECTED";
  }
  
  Serial.println(response);
}

void handlePositioning(String position_type) {
  if (position_type == "PICKUP") {
    // Position robot optimally for pickup
    // Small forward movement for precision
    moveForwardPrecise(50); // 5cm forward
    Serial.println("NAV:SUCCESS:PICKUP_POSITION");
  }
  else if (position_type == "PLACE") {
    // Position robot optimally for placement
    moveForwardPrecise(100); // 10cm forward
    Serial.println("NAV:SUCCESS:PLACE_POSITION");
  }
  else {
    Serial.println("NAV:ERROR:INVALID_POSITION_TYPE");
  }
}

void handleReset() {
  // Reset encoders and position
  left_encoder_count = 0;
  right_encoder_count = 0;
  robot_x = 0.0;
  robot_y = 0.0;
  robot_angle = 0.0;
  
  stopMotors();
  Serial.println("NAV:SUCCESS:RESET_COMPLETE");
}

void moveForward(int distance_mm) {
  long target_pulses = (distance_mm * PULSES_PER_REV) / (PI * WHEEL_DIAMETER);
  
  // Reset encoder counts
  left_encoder_count = 0;
  right_encoder_count = 0;
  
  // Start moving forward
  setMotorSpeeds(200, 200); // Medium speed
  
  // Monitor movement with obstacle detection
  while (abs(left_encoder_count) < target_pulses && abs(right_encoder_count) < target_pulses) {
    // Check for obstacles
    float front_distance = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
    
    if (front_distance < 15.0) { // 15cm obstacle threshold
      stopMotors();
      Serial.println("NAV:WARNING:OBSTACLE_DETECTED");
      return;
    }
    
    // PID control for straight movement
    pidControl();
    delay(10);
  }
  
  stopMotors();
  updatePosition(distance_mm, 0); // Update robot position
}

void moveBackward(int distance_mm) {
  long target_pulses = (distance_mm * PULSES_PER_REV) / (PI * WHEEL_DIAMETER);
  
  left_encoder_count = 0;
  right_encoder_count = 0;
  
  setMotorSpeeds(-150, -150); // Slower reverse speed
  
  while (abs(left_encoder_count) < target_pulses && abs(right_encoder_count) < target_pulses) {
    pidControl();
    delay(10);
  }
  
  stopMotors();
  updatePosition(-distance_mm, 0);
}

void strafeLeft(int distance_mm) {
  // For mecanum wheels or similar - adjust for your robot type
  // This is a placeholder for differential drive
  turnLeft(90);
  moveForward(distance_mm);
  turnRight(90);
}

void strafeRight(int distance_mm) {
  // For mecanum wheels or similar - adjust for your robot type
  turnRight(90);
  moveForward(distance_mm);
  turnLeft(90);
}

void turnLeft(int angle_degrees) {
  // Calculate required wheel rotations for turn
  float arc_length = (angle_degrees * PI * WHEEL_BASE) / 360.0;
  long target_pulses = (arc_length * PULSES_PER_REV) / (PI * WHEEL_DIAMETER);
  
  left_encoder_count = 0;
  right_encoder_count = 0;
  
  // Turn by rotating wheels in opposite directions
  setMotorSpeeds(-120, 120); // Left wheel backward, right wheel forward
  
  while (abs(left_encoder_count) < target_pulses || abs(right_encoder_count) < target_pulses) {
    delay(10);
  }
  
  stopMotors();
  robot_angle += angle_degrees;
  if (robot_angle >= 360) robot_angle -= 360;
}

void turnRight(int angle_degrees) {
  float arc_length = (angle_degrees * PI * WHEEL_BASE) / 360.0;
  long target_pulses = (arc_length * PULSES_PER_REV) / (PI * WHEEL_DIAMETER);
  
  left_encoder_count = 0;
  right_encoder_count = 0;
  
  setMotorSpeeds(120, -120); // Left wheel forward, right wheel backward
  
  while (abs(left_encoder_count) < target_pulses || abs(right_encoder_count) < target_pulses) {
    delay(10);
  }
  
  stopMotors();
  robot_angle -= angle_degrees;
  if (robot_angle < 0) robot_angle += 360;
}

void moveForwardPrecise(int distance_mm) {
  // Slow, precise movement for positioning
  long target_pulses = (distance_mm * PULSES_PER_REV) / (PI * WHEEL_DIAMETER);
  
  left_encoder_count = 0;
  right_encoder_count = 0;
  
  setMotorSpeeds(80, 80); // Very slow speed for precision
  
  while (abs(left_encoder_count) < target_pulses && abs(right_encoder_count) < target_pulses) {
    pidControl();
    delay(20); // Slower update rate for precision
  }
  
  stopMotors();
  updatePosition(distance_mm, 0);
}

void setMotorSpeeds(int left_speed, int right_speed) {
  // Constrain speeds to valid PWM range
  left_speed = constrain(left_speed, -MAX_SPEED, MAX_SPEED);
  right_speed = constrain(right_speed, -MAX_SPEED, MAX_SPEED);
  
  // Left motor control
  if (left_speed > 0) {
    digitalWrite(MOTOR_LEFT_DIR1, HIGH);
    digitalWrite(MOTOR_LEFT_DIR2, LOW);
    analogWrite(MOTOR_LEFT_PWM, left_speed);
  } else if (left_speed < 0) {
    digitalWrite(MOTOR_LEFT_DIR1, LOW);
    digitalWrite(MOTOR_LEFT_DIR2, HIGH);
    analogWrite(MOTOR_LEFT_PWM, -left_speed);
  } else {
    digitalWrite(MOTOR_LEFT_DIR1, LOW);
    digitalWrite(MOTOR_LEFT_DIR2, LOW);
    analogWrite(MOTOR_LEFT_PWM, 0);
  }
  
  // Right motor control
  if (right_speed > 0) {
    digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
    digitalWrite(MOTOR_RIGHT_DIR2, LOW);
    analogWrite(MOTOR_RIGHT_PWM, right_speed);
  } else if (right_speed < 0) {
    digitalWrite(MOTOR_RIGHT_DIR1, LOW);
    digitalWrite(MOTOR_RIGHT_DIR2, HIGH);
    analogWrite(MOTOR_RIGHT_PWM, -right_speed);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR1, LOW);
    digitalWrite(MOTOR_RIGHT_DIR2, LOW);
    analogWrite(MOTOR_RIGHT_PWM, 0);
  }
}

void stopMotors() {
  analogWrite(MOTOR_LEFT_PWM, 0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
  digitalWrite(MOTOR_LEFT_DIR1, LOW);
  digitalWrite(MOTOR_LEFT_DIR2, LOW);
  digitalWrite(MOTOR_RIGHT_DIR1, LOW);
  digitalWrite(MOTOR_RIGHT_DIR2, LOW);
}

void pidControl() {
  // Simple PID control to keep robot moving straight
  float left_speed_current = left_encoder_count;
  float right_speed_current = right_encoder_count;
  
  float error = left_speed_current - right_speed_current;
  
  left_error_sum += error;
  float left_derivative = error - left_last_error;
  
  float left_correction = kp * error + ki * left_error_sum + kd * left_derivative;
  
  // Apply corrections (simplified)
  if (abs(error) > 2) { // Only correct if significant difference
    if (error > 0) {
      // Left wheel is ahead, slow it down slightly
      // This is a simplified correction - adjust based on your motor setup
    } else {
      // Right wheel is ahead, slow it down slightly
    }
  }
  
  left_last_error = error;
}

float readUltrasonic(int trig_pin, int echo_pin) {
  // Send ultrasonic pulse
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);
  
  // Read echo pulse duration
  long duration = pulseIn(echo_pin, HIGH, 30000); // 30ms timeout
  
  if (duration == 0) {
    return 999.0; // No echo received, assume no obstacle
  }
  
  // Calculate distance in cm
  float distance = duration * 0.034 / 2;
  return distance;
}

void updatePosition(float delta_x, float delta_y) {
  // Update robot position based on movement
  float angle_rad = robot_angle * PI / 180.0;
  
  robot_x += delta_x * cos(angle_rad) - delta_y * sin(angle_rad);
  robot_y += delta_x * sin(angle_rad) + delta_y * cos(angle_rad);
}

// Encoder interrupt service routines
void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(MOTOR_LEFT_DIR1) == HIGH) {
    left_encoder_count++;
  } else {
    left_encoder_count--;
  }
}

void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(MOTOR_RIGHT_DIR1) == HIGH) {
    right_encoder_count++;
  } else {
    right_encoder_count--;
  }
}