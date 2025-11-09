#include <ESP32Servo.h>

int motor1Pin1 = 26; 
int motor1Pin2 = 25; 
int motor2Pin1 = 33; 
int motor2Pin2 = 32; 

const int pwmChannelA = 0;
const int pwmChannelB = 1;

Servo servoSensor;
#define TRIG_PIN 25
#define ECHO_PIN 26
#define servoPin 4

long duration;
int distance;
int16_t potitionServo;

// void forward(int speed) {
//   Serial.println("Moving Forward");
//   ledcWrite(pwmChannelA1, 0);        // Motor1 reverse = off
//   ledcWrite(pwmChannelA2, speed);    // Motor1 forward = PWM
//   ledcWrite(pwmChannelB1, 0);        // Motor2 reverse = off
//   ledcWrite(pwmChannelB2, speed);    // Motor2 forward = PWM
// }

// void backward(int speed) {
//   Serial.println("Moving Backward");
//   ledcWrite(pwmChannelA1, speed);    // Motor1 backward = PWM
//   ledcWrite(pwmChannelA2, 0);
//   ledcWrite(pwmChannelB1, speed);    // Motor2 backward = PWM
//   ledcWrite(pwmChannelB2, 0);
// }

// void left(int speed) {
//   Serial.println("Turning Left");
//   // Left motor backward, right motor forward
//   ledcWrite(pwmChannelA1, speed);
//   ledcWrite(pwmChannelA2, 0);
//   ledcWrite(pwmChannelB1, 0);
//   ledcWrite(pwmChannelB2, speed);
// }

// void right(int speed) {
//   Serial.println("Turning Right");
//   // Left motor forward, right motor backward
//   ledcWrite(pwmChannelA1, 0);
//   ledcWrite(pwmChannelA2, speed);
//   ledcWrite(pwmChannelB1, speed);
//   ledcWrite(pwmChannelB2, 0);
// }

// void stopMotor() {
//   Serial.println("Motor Stopped");
//   ledcWrite(pwmChannelA1, 0);
//   ledcWrite(pwmChannelA2, 0);
//   ledcWrite(pwmChannelB1, 0);
//   ledcWrite(pwmChannelB2, 0);
// }

void setup() {
  servoSensor.attach(servoPin);
  Serial.begin(115200);
  servoSensor.write(90);
  potitionServo=90;

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // pinMode(motor1Pin1, OUTPUT);
  // pinMode(motor1Pin2, OUTPUT);
  // pinMode(motor2Pin1, OUTPUT);
  // pinMode(motor2Pin2, OUTPUT);
}

void loop() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.0344 / 2;
  Serial.printf(distance);

  // if (distance<=100){
  //     stopMotor()
  //   if (potitionServo==180){
  //     // potitionServo=0;
  //     // servoSensor.write(0);
  //     // delay(1000)
  //     // if (distance>100) {
  //     //   Serial.write("Ke Kanan")
  //     // }
  //     // delay(2000);
  //   }
  //   // }else {
  //   //   potitionServo=180;
  //   //   servoSensor.write(180);
  //   //   delay(1000)
  //   //   if (distance>100) {
  //   //     Serial.write("Ke Kanan")
  //   //   }
  //   //   delay(2000);
  //   // }
  // } else {
  //   potitionServo=90;
  //   servoSensor.write(90);
  //   forward(122)
  // }
}
