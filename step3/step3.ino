#include <ESP32Servo.h>

// Motor pins
int motor1Pin1 = 25;
int motor1Pin2 = 26;
int motor2Pin1 = 32;
int motor2Pin2 = 33;

// PWM channels for motor control
const int pwmChannelA1 = 0;
const int pwmChannelA2 = 1;
const int pwmChannelB1 = 2;
const int pwmChannelB2 = 3;

// Ultrasonic sensor and servo pins
#define TRIG_PIN 12
#define ECHO_PIN 14
#define servoPin 27

// Servo PWM channel (gunakan channel yang tidak bentrok)
#define SERVO_CHANNEL 8

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

// Function to scan with servo and move robot to safe direction
void performTurn(String direction) {
  Serial.println("Scanning " + direction + " - Robot will move after scan");

  // Robot berhenti total untuk scanning
  stopMotor();
  delay(300); // Tunggu robot benar-benar berhenti

  int safeDistance = OBSTACLE_THRESHOLD + 10; // Jarak aman yang lebih besar
  bool foundSafePath = false;

  if (direction == "left") {
    // Servo scan ke kiri (180 derajat)
    servoSensor.write(180);
    delay(1000);

    int leftDist = measureDistance();
    Serial.print("Left scan distance: ");
    Serial.print(leftDist);
    Serial.println(" cm");

    // Kembalikan servo ke tengah
    servoSensor.write(90);
    delay(500);

    // Jika aman di kiri, gerakkan robot ke kiri
    if (leftDist > safeDistance) {
      Serial.println("Left path safe - Turning robot left");
      left(TURN_SPEED);
      delay(800); // Durasi putaran 90 derajat
      stopMotor();
      foundSafePath = true;
    }

  } else if (direction == "right") {
    // Servo scan ke kanan (0 derajat)
    servoSensor.write(0);
    delay(1000);

    int rightDist = measureDistance();
    Serial.print("Right scan distance: ");
    Serial.print(rightDist);
    Serial.println(" cm");

    // Kembalikan servo ke tengah
    servoSensor.write(90);
    delay(500);

    // Jika aman di kanan, gerakkan robot ke kanan
    if (rightDist > safeDistance) {
      Serial.println("Right path safe - Turning robot right");
      right(TURN_SPEED);
      delay(800); // Durasi putaran 90 derajat
      stopMotor();
      foundSafePath = true;
    }
  }

  if (!foundSafePath) {
    Serial.println("No safe path found in scanned direction");
    // Robot tetap berhenti dan akan melakukan scanning lagi
  }
}

// Fungsi baru: scanning komprehensif dengan servo
void scanAndFindSafePath() {
  Serial.println("Comprehensive servo scanning - Finding safe path");
  stopMotor();
  delay(500);

  // Scan kiri
  servoSensor.write(180);
  delay(1000);
  int leftDist = measureDistance();
  Serial.print("Left: ");
  Serial.print(leftDist);
  Serial.println(" cm");

  // Scan tengah
  servoSensor.write(90);
  delay(500);
  int centerDist = measureDistance();
  Serial.print("Center: ");
  Serial.print(centerDist);
  Serial.println(" cm");

  // Scan kanan
  servoSensor.write(0);
  delay(1000);
  int rightDist = measureDistance();
  Serial.print("Right: ");
  Serial.print(rightDist);
  Serial.println(" cm");

  // Kembali ke tengah
  servoSensor.write(90);
  delay(500);

  int safeDistance = OBSTACLE_THRESHOLD + 10;

  // Prioritaskan jalan yang paling aman
  if (rightDist > safeDistance && rightDist >= leftDist) {
    Serial.println("Turning right (safest path)");
    right(TURN_SPEED);
    delay(800);
    stopMotor();
  } else if (leftDist > safeDistance) {
    Serial.println("Turning left (safest path)");
    left(TURN_SPEED);
    delay(800);
    stopMotor();
  } else {
    Serial.println("No safe path - All directions blocked");
    backward(REVERSE_SPEED);
    delay(1000);
    stopMotor();
  }

  scanningComplete = false;
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

  // Initialize servo dengan channel yang ditentukan
  servoSensor.setPeriodHertz(50); // Standard 50hz servo
  servoSensor.attach(servoPin, SERVO_CHANNEL, 500, 2400); // Min=500us, Max=2400us
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
  ledcAttach(motor1Pin1, pwmChannelA1);
  ledcAttach(motor1Pin2, pwmChannelA2);
  ledcAttach(motor2Pin1, pwmChannelB1);
  ledcAttach(motor2Pin2, pwmChannelB2);

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

    // Step 4: Gunakan scanning komprehensif dengan servo
    if (obstacleFront) {
      Serial.println("Obstacle detected! Using servo to find safe path...");
      scanAndFindSafePath();
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
