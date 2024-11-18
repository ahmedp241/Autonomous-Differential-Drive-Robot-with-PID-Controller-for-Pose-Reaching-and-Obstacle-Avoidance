#include <Servo.h>

// Pin Definitions for motors
const int IN1 = 6;  // Motor A direction pin 1
const int IN2 = 7;  // Motor A direction pin 2
const int IN3 = 8;  // Motor B direction pin 1
const int IN4 = 9;  // Motor B direction pin 2
const int ENA = 10; // Motor A speed control pin (PWM)
const int ENB = 11; // Motor B speed control pin (PWM)

// Pin Definitions for Ultrasonic Sensor
#define trigPin 12   // Trigger pin for ultrasonic sensor
#define echoPin 13   // Echo pin for ultrasonic sensor

// Pin Definition for Servo
#define servoPin 5   // Servo motor control pin

// Encoder Pins
const int enc_left = 2;    // Left wheel encoder pin
const int enc_right = 4;   // Right wheel encoder pin

// Variables to store encoder values
volatile int left_encoder_count = 0;
volatile int right_encoder_count = 0;

const int encoder_counts_per_rev = 20; // Example value
const int thresholdDistance = 20;      // Threshold for obstacle detection in cm

// Ultrasonic sensor variables
long duration;
int distance;

// Wheel and robot geometry
const float R = 12.7;  // Wheel radius in cm
const float L = 50;    // Distance between wheels in cm

// Servo motor control
Servo myServo;
int servoPos = 90; // Starting position for servo (center)

// PID gains
float Kp_x = 3.5, Ki_x = 0.5, Kd_x = 0.1;
float Kp_y = 8.0, Ki_y = 3.45, Kd_y = 0.7;
float Kp_theta = 0.02, Ki_theta = 0.4, Kd_theta = 0.65;

// State variables
float x = 0.0, y = 0.0, theta = 0.0;
float e_x = 0.0, e_y = 0.0, e_theta = 0.0;

// Define constants
const int MAX_VOLTAGE = 255; // Max PWM value

// Time and integral terms for PID
float integral_x = 0.0, integral_y = 0.0, integral_theta = 0.0;
float prev_e_x = 0.0, prev_e_y = 0.0, prev_e_theta = 0.0;
unsigned long lastTime = 0;

// Interrupt Service Routines for encoders
void leftEncoderISR() {
  left_encoder_count++;
}

void rightEncoderISR() {
  right_encoder_count++;
}

// Setup Function
void setup() {
  Serial.begin(9600);
  lastTime = millis();

  // Set up motor control pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Set up encoder pins
  pinMode(enc_left, INPUT);
  pinMode(enc_right, INPUT);
  attachInterrupt(digitalPinToInterrupt(enc_left), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(enc_right), rightEncoderISR, RISING);

  // Set up ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initialize servo motor
  myServo.attach(servoPin);
  myServo.write(servoPos);  // Center position
}

// Main loop
void loop() {
  // Desired pose
  float x_d = 100.0, y_d = 100.0, theta_d = PI / 4;  // Example

  // Get current angular velocities from encoders
  float omega_left = readOmega(enc_left, left_encoder_count);
  float omega_right = readOmega(enc_right, right_encoder_count);

  // Check for obstacles
  distance = getDistance();
  if (distance < thresholdDistance) {
    avoidObstacle();
  } else {
    float u_left, u_right;
    regulateMotors(x_d, y_d, theta_d, omega_left, omega_right, u_left, u_right);

    // Control motor directions and speed
    setMotorDirection(u_left, u_right);
    analogWrite(ENA, abs(u_left));   // Control left motor speed
    analogWrite(ENB, abs(u_right));  // Control right motor speed
  }
}

// Get distance from ultrasonic sensor
long getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;  // Convert to cm
}

// Avoid obstacles
void avoidObstacle() {
  myServo.write(45);  // Check left
  delay(500);
  long leftDistance = getDistance();

  myServo.write(135); // Check right
  delay(500);
  long rightDistance = getDistance();

  myServo.write(90);  // Reset to center

  if (leftDistance < 20 && rightDistance >= 20) {
    // Move right
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  } else if (rightDistance < 20 && leftDistance >= 20) {
    // Move left
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {
    // Move forward
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  }

  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
  delay(1000);
  stopMotors();
}

// Stop motors
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// Regulate motor tensions using PID
void regulateMotors(float x_d, float y_d, float theta_d, 
                    float omega_left, float omega_right, 
                    float &u_left, float &u_right) {
  calculatePose(omega_left, omega_right, x, y, theta);
  calculateErrors(x_d, y_d, theta_d);
  pidControl(u_left, u_right);
}

// Calculate the robot's pose
void calculatePose(float omega_left, float omega_right, float &x, float &y, float &theta) {
  float v = (R / 2) * (omega_right + omega_left); // Linear velocity
  float omega = (R / L) * (omega_right - omega_left); // Angular velocity

  x += v * cos(theta) * (millis() - lastTime) / 1000.0;
  y += v * sin(theta) * (millis() - lastTime) / 1000.0;
  theta += omega * (millis() - lastTime) / 1000.0;

  if (theta > PI) theta -= 2 * PI;
  if (theta < -PI) theta += 2 * PI;
}

// Calculate errors for PID
void calculateErrors(float x_d, float y_d, float theta_d) {
  e_x = x_d - x;
  e_y = y_d - y;
  e_theta = theta_d - theta;
}

// PID controller
void pidControl(float &u_left, float &u_right) {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // Time step in seconds

  integral_x += e_x * deltaTime;
  integral_y += e_y * deltaTime;
  integral_theta += e_theta * deltaTime;

  float derivative_x = (e_x - prev_e_x) / deltaTime;
  float derivative_y = (e_y - prev_e_y) / deltaTime;
  float derivative_theta = (e_theta - prev_e_theta) / deltaTime;

  float u1 = Kp_x * e_x + Ki_x * integral_x + Kd_x * derivative_x;
  float u2 = Kp_y * e_y + Ki_y * integral_y + Kd_y * derivative_y;
  float u3 = Kp_theta * e_theta + Ki_theta * integral_theta + Kd_theta * derivative_theta;

  u_left = u1 - u2 - u3;
  u_right = u1 + u2 + u3;

  u_left = clamp(u_left, -MAX_VOLTAGE, MAX_VOLTAGE);
  u_right = clamp(u_right, -MAX_VOLTAGE, MAX_VOLTAGE);

  prev_e_x = e_x;
  prev_e_y = e_y;
  prev_e_theta = e_theta;

  lastTime = currentTime;
}

// Clamp function to limit values
float clamp(float value, float minVal, float maxVal) {
  if (value > maxVal) return maxVal;
  if (value < minVal) return minVal;
  return value;
}

float readOmega(int enc_pin, volatile int &encoder_count) {
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();
  float deltaT = (currentTime - lastTime) / 1000.0; // seconds

  float omega = (encoder_count * 2 * PI) / (encoder_counts_per_rev * deltaT);
  encoder_count = 0;
  lastTime = currentTime;

  return omega;
}

// Set motor direction based on control signals
void setMotorDirection(float u_left, float u_right) {
  if (u_left > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }

  if (u_right > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
}
