#include <IBusBM.h>
#include <Servo.h>

#define SERVO_RIGHT 9
#define SERVO_LEFT 8
#define MOTOR_PIN 7 // Define the motor control pin

IBusBM IBus;
Servo ServoRight;
Servo ServoLeft;
Servo Motor; // Create a servo object for the motor (ESC)

int CH1 = 0;
int CH2 = 0;
int CH3 = 0;
int CH4 = 0;
int CH5 = 0;
int CH6 = 0;

void setup() {
  Serial.begin(115200);
  IBus.begin(Serial);
  ServoRight.attach(SERVO_RIGHT);
  ServoLeft.attach(SERVO_LEFT);
  Motor.attach(MOTOR_PIN); // Attach the motor to the control pin
  ServoRight.write(120);
  ServoLeft.write(120);
  Motor.write(0); // Initialize the motor to zero throttle
}

void loop() { 
  CH1 = IBus.readChannel(0);
  CH2 = IBus.readChannel(1);
  CH3 = IBus.readChannel(2);
  CH4 = IBus.readChannel(3);
  CH5 = IBus.readChannel(4);
  CH6 = IBus.readChannel(5);

  int servoRightAngle = map(CH1, 1000, 2000, 0, 180);
  int servoLeftAngle = 180 - servoRightAngle;
  int motorSpeed = map(CH3, 1000, 2000, 0, 180); // Map the channel value to motor speed range

  ServoRight.write(servoRightAngle);
  ServoLeft.write(servoLeftAngle);
  Motor.write(motorSpeed); // Write the mapped value to the motor

  Serial.print("R servo position: ");
  Serial.print(servoRightAngle);
  Serial.print(" L servo position: ");
  Serial.print(servoLeftAngle);
  Serial.print(" Motor speed: ");
  Serial.println(motorSpeed);

  delay(20);
}