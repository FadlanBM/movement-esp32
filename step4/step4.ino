#include <ESP32Servo.h>

int motor1Pin1 = 26;
int motor1Pin2 = 25;
int motor2Pin1 = 33;
int motor2Pin2 = 32; 

const int pwmChannelA1 = 0;
const int pwmChannelA2 = 1;
const int pwmChannelB1 = 2;
const int pwmChannelB2 = 3;

Servo servoSensor;
#define TRIG_PIN 27
#define ECHO_PIN 14
#define servoPin 2

long duration;
int distance;
int16_t potitionServo;
bool isScanning = false;
int servoDirection = 1;
int safeAngle = -1;
int bestSafeAngle = -1;
int maxSafeDistance = 0;
bool scanCompleted = false;
unsigned long lastScanTime = 0;
unsigned long lastMoveTime = 0;
const int SCAN_DELAY = 30;
const int SERVO_STEP = 2;
const int OBSTACLE_THRESHOLD = 40;
const int SAFE_DISTANCE = 100;
const int CRITICAL_DISTANCE = 20;

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

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing robot...");

  // Set ESP32 timer untuk servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Setup servo dengan frekuensi standar
  servoSensor.setPeriodHertz(50);
  servoSensor.attach(servoPin, 500, 2400); // min/max pulse width

  // Test servo movement
  testServo();
  potitionServo = 90;

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  ledcSetup(pwmChannelA1, 5000, 8);
  ledcSetup(pwmChannelA2, 5000, 8);
  ledcSetup(pwmChannelB1, 5000, 8);
  ledcSetup(pwmChannelB2, 5000, 8);

  ledcAttachPin(motor1Pin1, pwmChannelA1);
  ledcAttachPin(motor1Pin2, pwmChannelA2);
  ledcAttachPin(motor2Pin1, pwmChannelB1);
  ledcAttachPin(motor2Pin2, pwmChannelB2);
}

int readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH, 30000);
  return duration * 0.0344 / 2;
}

void testServo() {
  Serial.println("=== SERVO TEST ===");
  Serial.print("Attached to pin: ");
  Serial.println(servoPin);

  servoSensor.write(0);
  Serial.println("Servo at 0°");
  delay(1000);

  servoSensor.write(90);
  Serial.println("Servo at 90°");
  delay(1000);

  servoSensor.write(180);
  Serial.println("Servo at 180°");
  delay(1000);

  servoSensor.write(90);
  Serial.println("Servo back to 90°");
  delay(1000);

  Serial.println("=== SERVO TEST COMPLETE ===");
}

void loop() {
  distance = readDistance();

  // Emergency stop untuk objek terlalu dekat
  if (distance <= CRITICAL_DISTANCE && distance > 0) {
    Serial.println("EMERGENCY STOP - Objek terlalu dekat!");
    stopMotor();
    if (!isScanning) {
      isScanning = true;
      scanCompleted = false;
      potitionServo = 0;  // Mulai dari kiri
      servoDirection = 1; // Bergerak ke kanan
      safeAngle = -1;
      bestSafeAngle = -1;
      maxSafeDistance = 0;
      lastScanTime = millis();

      // Pindahkan servo ke posisi awal
      Serial.println("Emergency: Moving servo to start position (0°)");
      servoSensor.write(0);
      delay(300);
    }
  }
  // Deteksi objek dalam jarak berbahaya
  else if (distance <= OBSTACLE_THRESHOLD && distance > 0 && !isScanning) {
    Serial.println("Objek terdeteksi! Mulai scanning dari kiri ke kanan...");
    stopMotor();
    isScanning = true;
    scanCompleted = false;
    potitionServo = 0;  // Mulai dari kiri
    servoDirection = 1; // Bergerak ke kanan
    safeAngle = -1;
    bestSafeAngle = -1;
    maxSafeDistance = 0;
    lastScanTime = millis();

    // Pindahkan servo ke posisi awal (kiri)
    Serial.println("Moving servo to start position (0°)");
    servoSensor.write(0);
    delay(300);
  }
  else if (isScanning && millis() - lastScanTime >= SCAN_DELAY) {
    lastScanTime = millis();

    // Gerakkan servo
    potitionServo += (servoDirection * SERVO_STEP);

    Serial.print("Scanning - Current position: ");
    Serial.print(potitionServo);
    Serial.print("°, Direction: ");
    Serial.println(servoDirection == 1 ? "Right" : "Left");

    // Balik arah di batas
    if (potitionServo >= 180) {
      potitionServo = 180;
      servoDirection = -1;
    }
    else if (potitionServo <= 0) {
      potitionServo = 0;
      servoDirection = 1;
    }

    Serial.print("Moving servo to: ");
    Serial.println(potitionServo);
    servoSensor.write(potitionServo);
    delay(50); // Beri waktu servo untuk stabil

    // Baca jarak di posisi servo saat ini
    int currentDistance = readDistance();

    Serial.print("Servo: ");
    Serial.print(potitionServo);
    Serial.print("°, Jarak: ");
    Serial.print(currentDistance);
    Serial.print("cm");

    // Cari jalur terbaik dengan jarak terjauh
    if (currentDistance > SAFE_DISTANCE && currentDistance > maxSafeDistance) {
      maxSafeDistance = currentDistance;
      bestSafeAngle = potitionServo;
      Serial.print(" -> BEST! Max: ");
      Serial.print(maxSafeDistance);
      Serial.print("cm at ");
      Serial.print(bestSafeAngle);
      Serial.println("°");
    }
    else if (currentDistance > OBSTACLE_THRESHOLD && safeAngle == -1) {
      safeAngle = potitionServo;
      Serial.print(" -> Safe path found at ");
      Serial.print(safeAngle);
      Serial.println("°");
    }
    else {
      Serial.println();
    }

    // Selesai scanning setelah satu cycle penuh (kembali ke 0° setelah mencapai 180°)
    if (potitionServo == 0 && servoDirection == 1 && !scanCompleted) {
      Serial.println("=== SCANNING COMPLETED - ANALYZING RESULTS ===");
      Serial.print("Best angle found: ");
      Serial.print(bestSafeAngle);
      Serial.print("° with distance ");
      Serial.print(maxSafeDistance);
      Serial.println("cm");

      scanCompleted = true;
      isScanning = false;

      // Prioritaskan jalur terbaik (terjauh)
      int targetAngle = (bestSafeAngle != -1) ? bestSafeAngle : safeAngle;

      if (targetAngle != -1) {
        Serial.print("Memilih jalur di ");
        Serial.print(targetAngle);
        Serial.print("° dengan jarak ");
        Serial.print(maxSafeDistance > 0 ? maxSafeDistance : SAFE_DISTANCE);
        Serial.println("cm");

        // Decision making berdasarkan posisi terbaik
        if (targetAngle < 70) {
          // Belok tajam kanan
          Serial.println("Belok tajam kanan");
          right(150);
          delay(600);
        }
        else if (targetAngle < 110) {
          // Maju lurus atau sedikit belok
          if (targetAngle < 85) {
            Serial.println("Belok kanan ringan");
            right(100);
            delay(400);
          }
          else if (targetAngle > 95) {
            Serial.println("Belok kiri ringan");
            left(100);
            delay(400);
          }
          else {
            Serial.println("Maju lurus - jalan aman di depan");
            forward(122);
            delay(300);
          }
        }
        else {
          // Belok tajam kiri
          Serial.println("Belok tajam kiri");
          left(150);
          delay(600);
        }
      }
      else {
        // Tidak ada jalur aman - mundur dan putar
        Serial.println("Tidak ada jalan aman - mundur dan putar");
        backward(150);
        delay(800);
        right(150);
        delay(1000);
      }

      // Reset servo dan variabel
      Serial.println("Resetting servo to center position");
      potitionServo = 90;
      servoSensor.write(90);
      delay(200); // Extra delay untuk servo return
      safeAngle = -1;
      bestSafeAngle = -1;
      maxSafeDistance = 0;
      scanCompleted = false;
      lastMoveTime = millis();
    }
  }
  else if (distance > SAFE_DISTANCE && !isScanning && distance > 0) {
    // Tidak ada objek dalam jarak aman, maju dengan kecepatan adaptif
    int speed = 122;

    // Kurangi kecepatan jika objek mendekat
    if (distance < SAFE_DISTANCE + 50) {
      speed = map(distance, SAFE_DISTANCE, SAFE_DISTANCE + 50, 80, 122);
    }

    Serial.print("Maju lurus - Jarak: ");
    Serial.print(distance);
    Serial.print("cm, Speed: ");
    Serial.println(speed);

    forward(speed);
    lastMoveTime = millis();

    // Pastikan servo di tengah saat tidak scanning
    if (potitionServo != 90) {
      Serial.println("Returning servo to center (90°)");
      potitionServo = 90;
      servoSensor.write(90);
      delay(100);
    }
  }
  else if (distance == 0 || distance > 300) {
    // Handle sensor error atau pembacaan tidak valid
    Serial.println("Sensor error - stopping");
    stopMotor();
    if (!isScanning) {
      delay(100);
    }
  }

  // Auto-stop jika terlalu lama tidak bergerak (stuck detection)
  if (!isScanning && millis() - lastMoveTime > 5000) {
    Serial.println("Stuck detected - backup and turn");
    stopMotor();
    backward(100);
    delay(500);
    right(120);
    delay(700);
    lastMoveTime = millis();
  }
}
