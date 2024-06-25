#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>
#include <IBusBM.h>

MPU6050 mpu;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  
  while (!Serial); // Wait for serial port to connect

  mpu.initialize();
  
  // Optional: Configure MPU6050 settings
  // mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  // mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  // mpu.setDLPFMode(MPU6050_DLPF_BW_42);
}

void loop() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Serial.print("Accelerometer: ");
  // Serial.print("X = "); Serial.print(ax);
  // Serial.print(", Y = "); Serial.print(ay);
  // Serial.print(", Z = "); Serial.println(az);
  
  // Serial.print("Gyroscope: ");
  // Serial.print("X = "); Serial.print(gx);
  // Serial.print(", Y = "); Serial.print(gy);
  // Serial.print(", Z = "); Serial.println(gz);

  Serial.print(ax);
  Serial.print(","); Serial.print(ay);
  Serial.print(","); Serial.print(az);

  Serial.print(gx);
  Serial.print(","); Serial.print(gy);
  Serial.print(","); Serial.println(gz);
  
  delay(50); // Adjust delay as needed
}