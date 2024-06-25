#include <IBusBM.h>
#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>
#include <MadgwickAHRS.h>

#define SERVO_PIN 9
IBusBM IBus;
Servo myServo;
MPU6050 mpu;

int CH1 = 0;
int CH2 = 0;
int CH3 = 0;
int CH4 = 0;
int CH5 = 0;
int CH6 = 0;

Madgwick filter;
unsigned long lastTime = 0;
const float sensorRate = 0.01; // Sensor update rate

// Variables to store initial orientation
float initialRoll = 0.0;
float initialPitch = 0.0;
float initialYaw = 0.0;
bool initialized = false;

// Variables to store min and max values
float minRoll = 0.0, maxRoll = 0.0;
float minPitch = 0.0, maxPitch = 0.0;
float minYaw = 0.0, maxYaw = 0.0;

void setup() {
  Serial.begin(115200);
  IBus.begin(Serial);
  myServo.attach(SERVO_PIN);
  myServo.write(90); // Center the servo initially
  
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
  filter.begin(sensorRate);

  delay(1000);

  float roll, pitch, yaw;
  eulers(roll, pitch, yaw);
  initialRoll = roll;
  initialPitch = pitch;
  initialYaw = yaw;
  initialized = true;

  // Initialize min and max values with the initial orientation
  minRoll = maxRoll = initialRoll;
  minPitch = maxPitch = initialPitch;
  minYaw = maxYaw = initialYaw;
}

void loop() { 
  CH1 = IBus.readChannel(0);
  CH2 = IBus.readChannel(1);
  CH3 = IBus.readChannel(2);
  CH4 = IBus.readChannel(3);
  CH5 = IBus.readChannel(4);
  CH6 = IBus.readChannel(5);
  int servoAngle = map(CH1, 1000, 2000, 0, 180);

  myServo.write(servoAngle);

  float roll, pitch, yaw;
  eulers(roll, pitch, yaw);
  roll -= initialRoll;
  pitch -= initialPitch;
  yaw -= initialYaw;

  // Update min and max values
  updateMinMax(roll, pitch, yaw);

  Serial.print(roll);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.println(yaw);

  // Serial.print("Min Roll: ");
  // Serial.print(minRoll);
  // Serial.print(" Max Roll: ");
  // Serial.print(maxRoll);
  // Serial.print(" Min Pitch: ");
  // Serial.print(minPitch);
  // Serial.print(" Max Pitch: ");
  // Serial.print(maxPitch);
  // Serial.print(" Min Yaw: ");
  // Serial.print(minYaw);
  // Serial.print(" Max Yaw: ");
  // Serial.println(maxYaw);

  delay(10);
}

void eulers(float &roll, float &pitch, float &yaw) {
  // Get the time
  unsigned long now = millis();
  float deltaTime = (now - lastTime) / 1000.0f;
  lastTime = now;

  // Read raw accelerometer and gyroscope data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert to G's
  float axg = ax / 16384.0;
  float ayg = ay / 16384.0;
  float azg = az / 16384.0;

  // Convert to degrees/sec
  float gxds = gx / 131.0;
  float gyds = gy / 131.0;
  float gzds = gz / 131.0;

  // Update the filter, which computes the quaternion
  filter.updateIMU(gxds, gyds, gzds, axg, ayg, azg);

  // Convert to degrees
  roll = filter.getRoll() * 57.2958;
  pitch = filter.getPitch() * 57.2958;
  yaw = filter.getYaw() * 57.2958;
}

void updateMinMax(float roll, float pitch, float yaw) {
  // Update min and max roll
  if (roll < minRoll) minRoll = roll;
  if (roll > maxRoll) maxRoll = roll;
  
  // Update min and max pitch
  if (pitch < minPitch) minPitch = pitch;
  if (pitch > maxPitch) maxPitch = pitch;
  
  // Update min and max yaw
  if (yaw < minYaw) minYaw = yaw;
  if (yaw > maxYaw) maxYaw = yaw;
}
