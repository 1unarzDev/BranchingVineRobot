# Vine Robot Project

This repository contains the code for an ongoing project aimed at creating branching vine robots for navigating complex environments.

## Checklist

Here is a checklist of the tasks we need to accomplish:

- [ ] Create ROS base
- [ ] Get D435 Intel RealSense wrapper for ROS working
- [ ] Implement DBSCAN for depth clustering using Scikit Learn
- [ ] Write the proper networking code to connect the components together (Teensy 4.1, ESP32 POE, computer, and D435)
- [ ] Write the code to spin the servos from both the ESP32 POE board and PCA9685 connected to the Teensy
- [ ] Implement a decision/state machine that subscribes to other things and sends messages to the kinematic calculation script to output PWM
- [ ] Create code to calculate the kinematics

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
