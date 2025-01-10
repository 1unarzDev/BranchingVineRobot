#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40); // PCA9685 default I2C address

// Modify the widths below for the pulses to map properly depending on the servo
#define SERVOMIN 125  // Minimum pulse length
#define SERVOMAX 625  // Maximum pulse length

// Map angle to pulse width
int angleToPulse(int ang) {
    int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
    return pulse;
}

void moveMotor(int motor) {
    pca9685.setPWM(motor, 0, angleToPulse((rand() - RAND_MAX / 2) * 2 / RAND_MAX ));
}

void setup() {
    Serial.begin(9600); // Adjust the baud rate depending on your board (also in the plaformio.ini)
    pca9685.begin();
    pca9685.setPWMFreq(60); // Analog servos run at ~60 Hz updates
}