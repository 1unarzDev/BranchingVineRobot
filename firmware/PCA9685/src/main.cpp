#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40); // PCA9685 default I2C address

#define SERVOMIN 125  // Minimum pulse length
#define SERVOMAX 625  // Maximum pulse length

// Map angle to pulse width
int angleToPulse(int ang) {
    int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
    Serial.print("Angle: ");
    Serial.print(ang);
    Serial.print(" pulse: ");
    Serial.println(pulse);
    return pulse;
}

void setup() {
    Serial.begin(9600);
    Serial.println("16-channel Servo Test!");
    board1.begin();
    board1.setPWMFreq(60); // Analog servos run at ~60 Hz updates
}

void loop() {
    // Sweep angles from 0 to 180 for channels 0-7
    for (int angle = 0; angle <= 180; angle += 10) {
        for (int i = 0; i < 15; i++) {
            board1.setPWM(i, 0, angleToPulse(angle));
        }
    }
}