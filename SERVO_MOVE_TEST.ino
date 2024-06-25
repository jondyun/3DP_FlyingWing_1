#include <Servo.h>

#define IBUS_SERIAL Serial1 // Define the serial port for iBUS communication
#define IBUS_BAUDRATE 115200 // iBUS communication baud rate
#define IBUS_CHANNEL_SERVO 1 // iBUS channel connected to servo
#define SERVO_PIN 9 // Arduino pin connected to servo

Servo servo;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  IBUS_SERIAL.begin(IBUS_BAUDRATE);

  // Attach servo to specified pin
  servo.attach(SERVO_PIN);
}

void loop() {
  // Read iBUS data
  while (IBUS_SERIAL.available() >= 32) { // Make sure there's enough data
    if (IBUS_SERIAL.read() == 0x20) { // Check for iBUS start byte
      int servoValue = IBUS_SERIAL.read() | (IBUS_SERIAL.read() << 8); // Read servo value from specified channel
      int mappedValue = map(servoValue, 1000, 2000, 0, 180); // Map servo value to servo range
      servo.write(mappedValue); // Write mapped value to servo
    }
  }
}
