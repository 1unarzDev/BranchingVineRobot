#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40); // PCA9685 default I2C address

// Minimum and maximum pulse widths for modded continous open-loop control potentiometer-based and closed-loop control
#define OPENMIN 345.5
#define OPENMAX 365.5  
#define CLOSEDMIN 123
#define CLOSEDMAX 614
#define FIRST_PNEUMATIC_CONTROL_VALVE 12
#define LAST_ACTUATOR 15
#define MAX_ANGLE 180
#define MIN_ANGLE 0

void openLoopControl(int motor, float speed) {
  int pulse = map(speed, -1, 1, OPENMIN, OPENMAX); // Map speed to corresponding pulse length
  pca9685.setPWM(motor, 0, pulse); // Set PWM signal on motor controller
}

void closedLoopControl(int motor, int angle) {
  int pulse = map(angle, MIN_ANGLE, MAX_ANGLE, CLOSEDMIN, CLOSEDMAX);
  pca9685.setPWM(motor, 0, pulse);
}

void setup() {
  Serial.begin(115200); // Match this baud rate with the Python script
  pca9685.begin();
  pca9685.setPWMFreq(60); // Servo frequency

  for(int i = 0; i < 16; i++){
    openLoopControl(i, 0);
  }

  Serial.println("Arduino Ready");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read input until newline
    input.trim(); // Remove extra spaces and newlines

    if (input.length() > 0) {
      int motor, val;
      sscanf(input.c_str(), "%d %d", &motor, &val); // Parse input as two integers

      if (motor >= 0 && motor < FIRST_PNEUMATIC_CONTROL_VALVE && val >= -1 && val <= 1) { // Ensure valid values
        openLoopControl(motor, val);
        Serial.print("Motor ");
        Serial.print(motor);
        Serial.print(" set to speed ");
        Serial.println(val);
      } 
      else if (motor >= FIRST_PNEUMATIC_CONTROL_VALVE && motor <= LAST_ACTUATOR && val >= MIN_ANGLE && val <= MAX_ANGLE){
        openLoopControl(motor, val);
        Serial.print("Motor ");
        Serial.print(motor);
        Serial.print(" set to angle ");
        Serial.println(val);
      }
      else {
        Serial.println("Invalid command");
      }
    }
  }
}