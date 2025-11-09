#include <ESP32Servo.h>

// Motor pins
int motor1Pin1 = 26;
int motor1Pin2 = 25;
int motor2Pin1 = 33;
int motor2Pin2 = 32;

// PWM channels for motor control
const int pwmChannelA1 = 0;
const int pwmChannelA2 = 1;
const int pwmChannelB1 = 2;
const int pwmChannelB2 = 3;

// Ultrasonic sensor and servo pins
#define TRIG_PIN 5
#define ECHO_PIN 18
#define servoPin 4

// Distance thresholds (in cm)
#define OBSTACLE_THRESHOLD 20
#define SAFE_DISTANCE 30
#define TURN_DISTANCE 25

Servo servoSensor;

long duration;
int distance;
int16_t potitionServo;

// Robot state variables
bool obstacleFront = false;
bool obstacleRight = false;
bool obstacleLeft = false;
bool scanningComplete = false;
unsigned long lastScanTime = 0;
const unsigned long SCAN_DELAY = 500;

// Motor speed constants
const int NORMAL_SPEED = 180;
const int TURN_SPEED = 150;
const int REVERSE_SPEED = 120;

void forward(int speed) {
  Serial.println("Moving Forward");
  ledcWrite(pwmChannelA1, 0);        // Motor1 reverse = off
  ledcWrite(pwmChannelA2, speed);    // Motor1 forward = PWM
  ledcWrite(pwmChannelB1, 0);        // Motor2 reverse = off
  ledcWrite(pwmChannelB2, speed);    // Motor2 forward = PWM
}

void backward(int speed) {
  Serial.println("Moving Backward");
  ledcWrite(pwmChannelA1, speed);    // Motor1 backward = PWM
  ledcWrite(pwmChannelA2, 0);
  ledcWrite(pwmChannelB1, speed);    // Motor2 backward = PWM
  ledcWrite(pwmChannelB2, 0);
}

void left(int speed) {
  Serial.println("Turning Left");
  // Left motor backward, right motor forward
  ledcWrite(pwmChannelA1, speed);
  ledcWrite(pwmChannelA2, 0);
  ledcWrite(pwmChannelB1, 0);
  ledcWrite(pwmChannelB2, speed);
}

void right(int speed) {
  Serial.println("Turning Right");
  // Left motor forward, right motor backward
  ledcWrite(pwmChannelA1, 0);
  ledcWrite(pwmChannelA2, speed);
  ledcWrite(pwmChannelB1, speed);
  ledcWrite(pwmChannelB2, 0);
}

void stopMotor() {
  Serial.println("Motor Stopped");
  ledcWrite(pwmChannelA1, 0);
  ledcWrite(pwmChannelA2, 0);
  ledcWrite(pwmChannelB1, 0);
  ledcWrite(pwmChannelB2, 0);
}

// Function to measure distance from ultrasonic sensor
int measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  return duration * 0.0344 / 2;
}

// Function to scan in a specific direction
int scanDirection(int angle) {
  servoSensor.write(angle);
  delay(300); // Wait for servo to reach position
  int distance = measureDistance();
  delay(100); // Additional delay for stable reading
  return distance;
}

// Function to scan all three directions
void scanSurroundings() {
  Serial.println("Scanning surroundings...");

  // Scan front
  int frontDist = scanDirection(90);
  obstacleFront = (frontDist <= OBSTACLE_THRESHOLD);
  Serial.print("Front distance: ");
  Serial.print(frontDist);
  Serial.print(" cm - Obstacle: ");
  Serial.println(obstacleFront ? "YES" : "NO");

  if (obstacleFront) {
    // Scan right
    int rightDist = scanDirection(0);
    obstacleRight = (rightDist <= OBSTACLE_THRESHOLD);
    Serial.print("Right distance: ");
    Serial.print(rightDist);
    Serial.print(" cm - Obstacle: ");
    Serial.println(obstacleRight ? "YES" : "NO");

    // Scan left
    int leftDist = scanDirection(180);
    obstacleLeft = (leftDist <= OBSTACLE_THRESHOLD);
    Serial.print("Left distance: ");
    Serial.print(leftDist);
    Serial.print(" cm - Obstacle: ");
    Serial.println(obstacleLeft ? "YES" : "NO");

    // Return servo to center
    servoSensor.write(90);
    delay(300);
  }

  scanningComplete = true;
  lastScanTime = millis();
}

// Function to perform turning maneuver
void performTurn(String direction) {
  Serial.println("Turning " + direction);

  if (direction == "left") {
    left(TURN_SPEED);
    delay(800); // Adjust duration for 90-degree turn
  } else if (direction == "right") {
    right(TURN_SPEED);
    delay(800); // Adjust duration for 90-degree turn
  }

  stopMotor();
  delay(200);
}

// Function to reverse and re-evaluate
void reverseAndRecheck() {
  Serial.println("Reversing - All directions blocked");
  backward(REVERSE_SPEED);
  delay(1500); // Reverse for 1.5 seconds
  stopMotor();
  delay(500);

  // Reset scanning state to trigger rescan
  scanningComplete = false;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Robot Obstacle Avoidance System Starting...");

  // Initialize servo
  servoSensor.attach(servoPin);
  servoSensor.write(90);
  potitionServo = 90;

  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize motor pins
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  // Setup PWM channels for motor control
  ledcSetup(pwmChannelA1, 5000, 8);
  ledcSetup(pwmChannelA2, 5000, 8);
  ledcSetup(pwmChannelB1, 5000, 8);
  ledcSetup(pwmChannelB2, 5000, 8);

  // Attach PWM channels to motor pins
  ledcAttachPin(motor1Pin1, pwmChannelA1);
  ledcAttachPin(motor1Pin2, pwmChannelA2);
  ledcAttachPin(motor2Pin1, pwmChannelB1);
  ledcAttachPin(motor2Pin2, pwmChannelB2);

  // Initialize robot state
  obstacleFront = false;
  obstacleRight = false;
  obstacleLeft = false;
  scanningComplete = false;

  Serial.println("Setup complete. Robot ready!");
}

void loop() {
  unsigned long currentTime = millis();

  // Step 1: Continuously check the front using ultrasonic sensor
  int frontDistance = measureDistance();
  Serial.print("Front distance: ");
  Serial.print(frontDistance);
  Serial.println(" cm");

  // Step 2: If obstacle detected in front, stop immediately
  if (frontDistance <= OBSTACLE_THRESHOLD) {
    stopMotor();
    Serial.println("Obstacle detected in front! Stopping immediately.");

    // Step 3: After stopping, check if there are obstacles on right and left
    if (!scanningComplete || (currentTime - lastScanTime > SCAN_DELAY)) {
      scanSurroundings();
    }

    // Step 4: Decision making based on scan results
    if (obstacleFront) {
      if (obstacleRight && !obstacleLeft) {
        // Right side blocked but left side clear: turn left
        Serial.println("Right blocked, left clear -> Turning left");
        performTurn("left");
        scanningComplete = false; // Reset for next scan
      } else if (obstacleLeft && !obstacleRight) {
        // Left side blocked but right side clear: turn right
        Serial.println("Left blocked, right clear -> Turning right");
        performTurn("right");
        scanningComplete = false; // Reset for next scan
      } else if (obstacleLeft && obstacleRight) {
        // Both right and left sides blocked, and still obstacle in front: reverse
        Serial.println("Both sides blocked, front still blocked -> Reversing");
        reverseAndRecheck();
      } else if (!obstacleLeft && !obstacleRight) {
        // Both sides clear - prefer turning right (or could choose based on other logic)
        Serial.println("Both sides clear - Choosing to turn right");
        performTurn("right");
        scanningComplete = false; // Reset for next scan
      }
    }
  } else {
    // No obstacle in front, move forward
    Serial.println("Path clear - Moving forward");
    forward(NORMAL_SPEED);

    // Reset scanning state for next obstacle encounter
    if (scanningComplete && (currentTime - lastScanTime > 2000)) {
      scanningComplete = false;
    }
  }

  // Small delay to prevent overwhelming the system
  delay(50);
}
