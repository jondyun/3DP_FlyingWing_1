#include <IBusBM.h>
#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>
#include <MadgwickAHRS.h>

#define SERVO_RIGHT 9
#define SERVO_LEFT 8
IBusBM IBus;
Servo ServoRight;
Servo ServoLeft;

int CH1 = 0;
int CH2 = 0;
int CH3 = 0;
int CH4 = 0;
int CH5 = 0;
int CH6 = 0;

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, accAngleZ; // Add Z-axis angle
float gyroAngleX, gyroAngleY, gyroAngleZ; // Add Z-axis angle
float angleX, angleY, angleZ; // Add Z-axis angle
float AccErrorX, AccErrorY, AccErrorZ; // Add Z-axis error
float GyroErrorX, GyroErrorY, GyroErrorZ; // Add Z-axis error
float elapsedTime, currentTime, previousTime;
int c = 0;

void setup() {
  Serial.begin(9600);

  // Initialize MPU6050
  initialize_MPU6050();
  
  // Calculate IMU error
  calculate_IMU_error();

  // Initialize iBus
  IBus.begin(Serial);
  
  // Attach servos and set them to the neutral position
  ServoRight.attach(SERVO_RIGHT);
  ServoLeft.attach(SERVO_LEFT);

  ServoRight.write(90); // Set right servo to neutral
  ServoLeft.write(90);  // Set left servo to neutral
}

void loop() {
  read_IMU();
  
  // Read iBus channels
  CH1 = IBus.readChannel(0);
  CH2 = IBus.readChannel(1);
  CH3 = IBus.readChannel(2);
  CH4 = IBus.readChannel(3);
  CH5 = IBus.readChannel(4);
  CH6 = IBus.readChannel(5);

  
  // Map iBus channel values to servo position range (0 to 180 degrees)
  int iBusServoPosition = map(CH1, 1000, 2000, 0, 180);
  iBusServoPosition = constrain(iBusServoPosition, 0, 180); // Ensure the value is within the valid range

  // Increase sensitivity by scaling angleX (you can adjust the scaling factor as needed)
  float scaledAngleX = angleX * 2.0; // Scale factor to make servos more sensitive to changes in angleX

  // Map the scaled angleX value to the servo position range (0 to 180 degrees)
  int gyroServoPosition = map(scaledAngleX, -90, 90, 0, 180);
  gyroServoPosition = constrain(gyroServoPosition, 0, 180); // Ensure the value is within the valid range
  
  // Combine iBus control with gyro control (you can adjust the ratio as needed)
  int combinedServoPositionRight = (iBusServoPosition + gyroServoPosition) / 2;
  int combinedServoPositionLeft = (iBusServoPosition - gyroServoPosition) / 2;
  
  combinedServoPositionRight = constrain(combinedServoPositionRight, 0, 180); // Ensure the value is within the valid range
  combinedServoPositionLeft = constrain(combinedServoPositionLeft, -180, 0); // Ensure the value is within the valid range

  // Move the servos based on the combined position, flipping directions
  ServoRight.write(180 - combinedServoPositionRight); // Adjust direction for right servo
  ServoLeft.write(combinedServoPositionLeft + 90);        // Adjust direction for left servo

  Serial.print(180 - combinedServoPositionRight);
  Serial.print(", ");
  Serial.println((combinedServoPositionLeft+90));
  
  delay(50);
}

void initialize_MPU6050() {
  Wire.begin();                      // Initialize communication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        // End the transmission
  // Configure Accelerometer
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  // Talk to the ACCEL_CONFIG register
  Wire.write(0x10);                  // Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(true);
}

void calculate_IMU_error() {
  // We can call this function in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can get the correct values.
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorZ = AccErrorZ + (AccZ - 1.0); // Assuming Z should be 1g if placed flat
    c++;
  }
  // Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  AccErrorZ = AccErrorZ / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); // Read 6 registers for gyro (2 for each axis)
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 32.8);
    GyroErrorY = GyroErrorY + (GyroY / 32.8);
    GyroErrorZ = GyroErrorZ + (GyroZ / 32.8);
    c++;
  }
  // Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
}

void read_IMU() {
  // === Read accelerometer data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  // For a range of +-8g, we need to divide the raw values by 4096, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 4096.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 4096.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 4096.0; // Z-axis value
  // Calculating angle values using
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) + 1.15; // AccErrorX ~(-1.15) See the calculate_IMU_error() custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - 0.52; // AccErrorY ~(0.5)
  accAngleZ = (atan(AccZ / sqrt(pow(AccX, 2) + pow(AccY, 2))) * 180 / PI) + 0.0; // AccErrorZ ~(0.0)
  // === Read gyro data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000.0;   // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 32.8; // For a 1000dps range we have to divide first the raw value by 32.8, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 32.8;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 32.8;
  GyroX = GyroX + GyroErrorX; // GyroErrorX
  GyroY = GyroY + GyroErrorY; // GyroErrorY
  GyroZ = GyroZ + GyroErrorZ; // GyroErrorZ
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by seconds (s) to get the angle in degrees
  gyroAngleX = GyroX * elapsedTime;
  gyroAngleY = GyroY * elapsedTime;
  gyroAngleZ = GyroZ * elapsedTime;
  // Complementary filter - combine accelerometer and gyro angle values
  angleX = 0.98 * (angleX + gyroAngleX) + 0.02 * accAngleX;
  angleY = 0.98 * (angleY + gyroAngleY) + 0.02 * accAngleY;
  angleZ = 0.98 * (angleZ + gyroAngleZ) + 0.02 * accAngleZ;
  delay(50);
}