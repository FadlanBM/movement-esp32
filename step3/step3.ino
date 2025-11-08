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
#define TRIG_PIN 25
#define ECHO_PIN 26
#define servoPin 4

long duration;
int distance;
int16_t potitionServo;
bool objectLeft = true;
bool objectRight = true;

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
  servoSensor.attach(servoPin);
  Serial.begin(115200);
  servoSensor.write(90);
  potitionServo=90;

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  // ledcSetup(pwmChannelA1, 5000, 8);
  // ledcSetup(pwmChannelA2, 5000, 8);
  // ledcSetup(pwmChannelB1, 5000, 8);
  // ledcSetup(pwmChannelB2, 5000, 8);

  // ledcAttachPin(motor1Pin1, pwmChannelA1);
  // ledcAttachPin(motor1Pin2, pwmChannelA2);
  // ledcAttachPin(motor2Pin1, pwmChannelB1);
  // ledcAttachPin(motor2Pin2, pwmChannelB2);
}

void loop() {
  // Baca jarak dari sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.0344 / 2;

  // Jika ada objek di depan dan belum pernah mengecek kanan/kiri
  if (distance <= 150 && objectRight && objectLeft) {
    Serial.println("Motor stop - Ada objek di depan");
    stopMotor();

    // Reset status objek
    objectRight = false;
    objectLeft = false;

    // Cek kanan terlebih dahulu
    potitionServo = 0;
    servoSensor.write(0);
    delay(1000);

    // Baca jarak di kanan
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    duration = pulseIn(ECHO_PIN, HIGH);
    int rightDistance = duration * 0.0344 / 2;

    if (rightDistance > 150) {
      // Kanan aman, langsung belok kanan
      Serial.println("Aman di kanan - Langsung belok kanan");
      potitionServo = 90;
      servoSensor.write(90);
      right(122);
      delay(500);
      objectRight = true;
      return; // Skip cek kiri
    }

    // Jika kanan tidak aman, cek kiri
    potitionServo = 180;
    servoSensor.write(180);
    delay(1000);

    // Baca jarak di kiri
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    duration = pulseIn(ECHO_PIN, HIGH);
    int leftDistance = duration * 0.0344 / 2;

    if (leftDistance > 150) {
      // Kiri aman, langsung belok kiri
      Serial.println("Aman di kiri - Langsung belok kiri");
      potitionServo = 90;
      servoSensor.write(90);
      left(122);
      delay(500);
      objectLeft = true;
      return;
    }

    // Keduanya tidak aman, mundur
    Serial.println("Tidak ada ruang kosong - Mundur");
    potitionServo = 90;
    servoSensor.write(90);
    backward(122);
    delay(1000);
  }else {
    // Tidak ada objek di depan, maju lurus
    potitionServo = 90;
    servoSensor.write(90);
    Serial.println("Ke Depan");
    forward(122);

    // Reset status untuk scanning berikutnya
    if (distance > 150) {
      objectRight = true;
      objectLeft = true;
    }
  }
}
