#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// Open loop motor control example for L298N board
#include <SimpleFOC.h>

#define IN1 1
#define IN2 2
#define IN3 3

// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(11);
// BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
BLDCDriver3PWM driver = BLDCDriver3PWM(IN1, IN2, IN3);

void setup() {
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // limiting motor movements
  motor.voltage_limit = 3;   // [V]
  motor.velocity_limit = 20; // [rad/s]

  // open loop control config
  motor.controller = MotionControlType::velocity_openloop;

  // init motor hardware
  motor.init();

  Serial.begin(115200);
  Serial.println("Motor ready!");
  _delay(1000);
}

float target_velocity = 15; // [rad/s]

void loop() {
  // open loop velocity movement
  // using motor.voltage_limit and motor.velocity_limit
  motor.move(target_velocity);
}