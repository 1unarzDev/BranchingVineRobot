# Vine Robot Project

This repository contains the code for an ongoing project aimed at creating branching vine robots for navigating complex environments. The current implementation uses Python for illustrative purposes, but may be updated to C++ for performance improvements in the future.

## Checklist

Here is a checklist of the tasks we need to accomplish:

- [x] Create ROS Humble base
- [x] Get D435 Intel RealSense wrapper for ROS working
- [x] Implement DBSCAN for depth clustering using Scikit Learn
- [x] Write the proper networking code to connect the components together (Teensy 4.1, ESP32 POE, computer, and D435)
- [x] Write the code to spin the servos from both the ESP32 POE board and PCA9685 connected to the Teensy
- [ ] Implement a decision/state machine that subscribes to other things and sends messages to the kinematic calculation script to output PWM
- [ ] Create code to calculate the kinematics
- [ ] Flesh out each of the previous components to create a functional robotic system

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) for details.
