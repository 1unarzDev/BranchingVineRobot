#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40); // PCA9685 default I2C address

#define SERVOMIN 345.5  // Minimum pulse length
#define SERVOMAX 365.5  // Maximum pulse length

void moveMotor(int motor, int speed) {
  int pulse = map(speed, -1, 1, SERVOMIN, SERVOMAX); // Map angle to corresponding pulse length
  pca9685.setPWM(motor, 0, pulse); // Set PWM signal on correct pin on motor controller 
}

void setup() {
  Serial.begin(115200); // Baud rate, match with ini file
  pca9685.begin();
  pca9685.setPWMFreq(60); // Servo communication frequency
}

void loop() {
  Serial.println("Turning counterclockwise");
  for(int i = 0; i < 16; i++){
    moveMotor(i, -1); 
  }
  delay(2000);

  Serial.println("Stopping");
  for(int i = 0; i < 16; i++){
    moveMotor(i, 0); 
  }
  delay(2000);

  Serial.println("Turning clockwise");
  for(int i = 0; i < 16; i++){
    moveMotor(i, 1); 
  }
  delay(2000);

  Serial.println("Stopping");
  for(int i = 0; i < 16; i++){
    moveMotor(i, 0); 
  }
  delay(2000);
}