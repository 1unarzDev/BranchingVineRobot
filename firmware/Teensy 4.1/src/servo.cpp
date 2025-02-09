// #include <Arduino.h>
// #include <SPI.h>
// #include <Wire.h>
// #include <Adafruit_PWMServoDriver.h>
// 
// Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40); // PCA9685 default I2C address
// 
// #define SERVOMIN 102  // Minimum pulse length
// #define SERVOMAX 512  // Maximum pulse length
// 
// void moveMotor(int motor, int angle) {
//   int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX); // Map angle to corresponding pulse length
//   pca9685.setPWM(motor, 0, pulse); // Set PWM signal on correct pin on motor controller 
// }
// 
// void setup() {
//   Serial.begin(115200); // Baud rate, match with ini file
//   pca9685.begin();
//   pca9685.setPWMFreq(50); // Servo communication frequency
// }
// 
// int angle = 0;
// void loop() {
//   if (Serial.available()) {
//     angle = Serial.parseInt();
//     moveMotor(11, angle);
//     Serial.print("Turned to angle ");
//     Serial.println(angle);
//   }
// }