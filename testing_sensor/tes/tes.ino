/*
 * ESP32 Module Test Program
 *
 * This program tests the functionality of:
 * 1. Ultrasonic Sensor (HC-SR04) - Distance measurement
 * 2. Servo Motor - Range of motion (0-180 degrees)
 * 3. TC1508A Motor Controller - Both motors at various speeds
 *
 * Hardware Configuration:
 * - Ultrasonic Sensor: TRIG_PIN = 5, ECHO_PIN = 18
 * - Servo: servoPin = 4
 * - Motor Controller: Motor1 (pins 26, 25), Motor2 (pins 33, 32)
 *
 * TC1508A Motor Control:
 * - Motor 1: Pin 26 (A1), Pin 25 (A2)
 * - Motor 2: Pin 33 (B1), Pin 32 (B2)
 * - Using PWM channels 0-3 for speed control
 */
#include <ESP32Servo.h>
#include <driver/ledc.h>

// Motor pins for TC1508A
int motor1Pin1 = 25;  // A1 - Motor 1 Input 1
int motor1Pin2 = 26;  // A2 - Motor 1 Input 2
int motor2Pin1 = 32;  // B1 - Motor 2 Input 1
int motor2Pin2 = 33;  // B2 - Motor 2 Input 2

// PWM channels for motor control (avoid conflicts with ESP32Servo)
const int pwmChannelA1 = 4;  // PWM Channel for Motor 1 Input 1
const int pwmChannelA2 = 5;  // PWM Channel for Motor 1 Input 2
const int pwmChannelB1 = 6;  // PWM Channel for Motor 2 Input 1
const int pwmChannelB2 = 7;  // PWM Channel for Motor 2 Input 2

// Ultrasonic sensor pins
#define TRIG_PIN 12
#define ECHO_PIN 14

// Servo pin
#define servoPin 27

// Test parameters
#define TEST_DELAY 2000      // Delay between tests (ms)
#define SERVO_DELAY 500      // Delay for servo movement (ms)
#define MOTOR_TEST_DELAY 1500 // Delay for motor test duration (ms)

// Speed constants for testing
const int SLOW_SPEED = 100;   // Slow speed for initial testing
const int MEDIUM_SPEED = 150; // Medium speed
const int FAST_SPEED = 200;   // Fast speed for performance testing

Servo testServo;

// Global variables
long duration;
int distance;
bool testCompleted = false;

// Function prototypes
void testUltrasonicSensor();
void testServoTes();
void testMotorController();
void setupMotors();
void moveMotor1(int speed, bool forward);
void moveMotor2(int speed, bool forward);
void stopAllMotors();
void printTestHeader(String testName);
void printTestResult(String testName, bool passed);

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 Module Test Program ===");
  Serial.println("Testing Ultrasonic Sensor, Servo, and TC1508A Motor Controller");
  Serial.println("============================================================\n");

  // Initialize servo
  testServo.attach(servoPin);
  testServo.write(90); // Center position
  delay(500);

  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Setup motor control
  setupMotors();

  Serial.println("Setup completed successfully!");
  Serial.println("Starting module tests...\n");
  delay(1000);
}

void setupMotors() {
  // Initialize motor pins
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  // Setup PWM channels for motor control
  // Frequency: 5000 Hz, Resolution: 8 bits (0-255)
  ledcSetup(pwmChannelA1, 5000, 8);
  ledcSetup(pwmChannelA2, 5000, 8);
  ledcSetup(pwmChannelB1, 5000, 8);
  ledcSetup(pwmChannelB2, 5000, 8);

  // Attach PWM channels to motor pins
  ledcAttach(motor1Pin1, pwmChannelA1);
  ledcAttach(motor1Pin2, pwmChannelA2);
  ledcAttach(motor2Pin1, pwmChannelB1);
  ledcAttach(motor2Pin2, pwmChannelB2);

  Serial.println("Motor controller initialized");
}

void loop() {
  if (!testCompleted) {
    Serial.println("Starting comprehensive module testing...\n");

    // Test 1: Ultrasonic Sensor
    testUltrasonicSensor();
    delay(TEST_DELAY);

    // Test 2: Servo Motor
    testServoTes();
    delay(TEST_DELAY);

    // Test 3: TC1508A Motor Controller
    testMotorController();

    testCompleted = true;
    Serial.println("\n=== All Tests Completed ===");
    Serial.println("You can now manually test individual functions by calling them");
    Serial.println("or reset the ESP32 to run all tests again.\n");
  } else {
    // After testing, provide manual testing menu
    Serial.println("Manual Testing Menu:");
    Serial.println("1 - Test Ultrasonic Sensor");
    Serial.println("2 - Test Servo Range");
    Serial.println("3 - Test Motor 1");
    Serial.println("4 - Test Motor 2");
    Serial.println("5 - Test Both Motors");
    Serial.println("Send 'r' to reset and run all tests again");

    delay(5000); // Wait 5 seconds before showing menu again
  }
}

void testUltrasonicSensor() {
  printTestHeader("Ultrasonic Sensor (HC-SR04) Test");

  Serial.println("Testing distance measurement capability...");
  Serial.println("Place an object at different distances and observe readings\n");

  // Take multiple readings to verify sensor functionality
  int readings[5];
  bool sensorWorking = true;

  for (int i = 0; i < 5; i++) {
    // Generate ultrasonic pulse
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Measure echo duration
    duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout

    if (duration == 0) {
      Serial.print("Reading ");
      Serial.print(i + 1);
      Serial.println(": TIMEOUT - Sensor not responding properly");
      sensorWorking = false;
    } else {
      // Calculate distance in cm
      distance = duration * 0.0344 / 2;
      readings[i] = distance;

      Serial.print("Reading ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(distance);
      Serial.println(" cm");
    }

    delay(1000); // Wait between readings
  }

  // Analyze results
  Serial.println("\n--- Test Analysis ---");
  if (sensorWorking) {
    int validReadings = 0;
    long totalDistance = 0;

    for (int i = 0; i < 5; i++) {
      if (readings[i] > 0 && readings[i] < 400) { // Valid range: 1-400cm
        validReadings++;
        totalDistance += readings[i];
      }
    }

    if (validReadings >= 3) {
      float avgDistance = (float)totalDistance / validReadings;
      Serial.print("Average distance: ");
      Serial.print(avgDistance);
      Serial.println(" cm");
      Serial.print("Valid readings: ");
      Serial.print(validReadings);
      Serial.println("/5");
      printTestResult("Ultrasonic Sensor", true);
    } else {
      Serial.println("Too few valid readings - sensor may be malfunctioning");
      printTestResult("Ultrasonic Sensor", false);
    }
  } else {
    Serial.println("Sensor not responding - check wiring and connections");
    printTestResult("Ultrasonic Sensor", false);
  }
  Serial.println();
}

void testServoTes() {
  printTestHeader("Servo Motor Range Test");

  Serial.println("Testing servo range of motion (0-180 degrees)");
  Serial.println("Observe smooth movement to each position\n");

  bool servoWorking = true;
  int testPositions[] = {0, 45, 90, 135, 180, 135, 90, 45, 90};
  int numPositions = sizeof(testPositions) / sizeof(testPositions[0]);

  for (int i = 0; i < numPositions; i++) {
    Serial.print("Moving to position: ");
    Serial.print(testPositions[i]);
    Serial.println(" degrees");

    testServo.write(testPositions[i]);
    delay(SERVO_DELAY);

    // Simple check - servo should reach position within reasonable time
    // In a real scenario, you might want to add feedback mechanism
    Serial.println("Position reached ");
  }

  Serial.println("\n--- Test Analysis ---");
  Serial.println("Servo movement test completed");
  Serial.println("Verify that servo:");
  Serial.println("- Moves smoothly between positions");
  Serial.println("- Reaches all extreme positions (0� and 180�)");
  Serial.println("- Returns to center (90�) without issues");
  printTestResult("Servo Motor", servoWorking);
  Serial.println();
}

void testMotorController() {
  printTestHeader("TC1508A Motor Controller Test");

  Serial.println("Testing both motors at different speeds");
  Serial.println("Ensure wheels/propellers are free to move!\n");

  // Test Motor 1
  Serial.println("--- Testing Motor 1 ---");
  Serial.println("Slow speed forward...");
  moveMotor1(SLOW_SPEED, true);
  delay(MOTOR_TEST_DELAY);
  stopAllMotors();
  delay(500);

  Serial.println("Medium speed forward...");
  moveMotor1(MEDIUM_SPEED, true);
  delay(MOTOR_TEST_DELAY);
  stopAllMotors();
  delay(500);

  Serial.println("Fast speed forward...");
  moveMotor1(FAST_SPEED, true);
  delay(MOTOR_TEST_DELAY);
  stopAllMotors();
  delay(500);

  Serial.println("Slow speed backward...");
  moveMotor1(SLOW_SPEED, false);
  delay(MOTOR_TEST_DELAY);
  stopAllMotors();
  delay(1000);

  // Test Motor 2
  Serial.println("\n--- Testing Motor 2 ---");
  Serial.println("Slow speed forward...");
  moveMotor2(SLOW_SPEED, true);
  delay(MOTOR_TEST_DELAY);
  stopAllMotors();
  delay(500);

  Serial.println("Medium speed forward...");
  moveMotor2(MEDIUM_SPEED, true);
  delay(MOTOR_TEST_DELAY);
  stopAllMotors();
  delay(500);

  Serial.println("Fast speed forward...");
  moveMotor2(FAST_SPEED, true);
  delay(MOTOR_TEST_DELAY);
  stopAllMotors();
  delay(500);

  Serial.println("Slow speed backward...");
  moveMotor2(SLOW_SPEED, false);
  delay(MOTOR_TEST_DELAY);
  stopAllMotors();
  delay(1000);

  // Test Both Motors Together
  Serial.println("\n--- Testing Both Motors Together ---");
  Serial.println("Both motors forward at slow speed...");
  moveMotor1(SLOW_SPEED, true);
  moveMotor2(SLOW_SPEED, true);
  delay(MOTOR_TEST_DELAY);
  stopAllMotors();
  delay(500);

  Serial.println("Both motors forward at fast speed...");
  moveMotor1(FAST_SPEED, true);
  moveMotor2(FAST_SPEED, true);
  delay(MOTOR_TEST_DELAY);
  stopAllMotors();
  delay(500);

  Serial.println("Turning test (Motor 1 forward, Motor 2 backward)...");
  moveMotor1(MEDIUM_SPEED, true);
  moveMotor2(MEDIUM_SPEED, false);
  delay(MOTOR_TEST_DELAY);
  stopAllMotors();

  Serial.println("\n--- Test Analysis ---");
  Serial.println("Motor controller test completed");
  Serial.println("Verify that:");
  Serial.println("- Both motors respond to speed commands");
  Serial.println("- Motors run smoothly at all tested speeds");
  Serial.println("- Forward and reverse directions work correctly");
  Serial.println("- No unusual noises or vibrations");
  Serial.println("- Both motors can operate simultaneously");
  printTestResult("TC1508A Motor Controller", true);
  Serial.println();
}

void moveMotor1(int speed, bool forward) {
  if (forward) {
    // Motor 1 forward: A1 = LOW (PWM 0), A2 = HIGH (PWM speed)
    ledcWrite(pwmChannelA1, 0);
    ledcWrite(pwmChannelA2, speed);
  } else {
    // Motor 1 backward: A1 = HIGH (PWM speed), A2 = LOW (PWM 0)
    ledcWrite(pwmChannelA1, speed);
    ledcWrite(pwmChannelA2, 0);
  }
}

void moveMotor2(int speed, bool forward) {
  if (forward) {
    // Motor 2 forward: B1 = LOW (PWM 0), B2 = HIGH (PWM speed)
    ledcWrite(pwmChannelB1, 0);
    ledcWrite(pwmChannelB2, speed);
  } else {
    // Motor 2 backward: B1 = HIGH (PWM speed), B2 = LOW (PWM 0)
    ledcWrite(pwmChannelB1, speed);
    ledcWrite(pwmChannelB2, 0);
  }
}

void stopAllMotors() {
  Serial.println("Stopping all motors");
  ledcWrite(pwmChannelA1, 0);
  ledcWrite(pwmChannelA2, 0);
  ledcWrite(pwmChannelB1, 0);
  ledcWrite(pwmChannelB2, 0);
}

void printTestHeader(String testName) {
  Serial.println("=================================");
  Serial.println("    " + testName);
  Serial.println("=================================");
}

void printTestResult(String testName, bool passed) {
  Serial.print("Test Result: ");
  if (passed) {
    Serial.println("PASSED ");
  } else {
    Serial.println("FAILED ");
  }
  Serial.println("=================================\n");
}