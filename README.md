# Branching Vine Robots

This repository contains the code for an ongoing project aimed at creating autonomous branching vine robots (BVRs) for navigating complex environments. The current architecture utilizes a network of ROS2 nodes, which ultimately communicate with a state machine that controls motor outputs. In order to determine the best possible path and when to branch, clusters of points within a calculated depth threhold are clustered by using the Scikit-Learn implementation of DBSCAN. 

## Code Structure

All code that is uploaded to hardware components such as the ESP32 POE, Teensy 4.1, Ardino Nano, etc. are stored within the `firmware` directory. All other ROS related code that runs on the primary processor is located in the `src\branching_vine_robot` directory as a package that can be placed into a ROS2 Humble workspace and compiled. The `src\interfaces` directory contains custom messages for concise packaging of information transfered between nodes. For ease of development, I opted for making a simple shell install script rather than using a Docker container because all processors used also double as a development environment for the prototyping phase.

In order to run multiple robots at once, the namespace feature of ROS2 Humble is utilized to isolate all nodes in each robot. There is a degree of hardcoding required (ip addresses to avoid clunky dhcp interactions, stero camera serial numbers, actuator motor positions, etc.); however, all constant values were cleanly packaged into `src\branching_vine_robot\config.py`.

## Useful Commands

```bash
colcon build --symlink-install # Do this in the directory containing src
source install/setup.bash # Source build ROS package, alternatively use setup.sh
ros2 launch branching_vine_robot multi.launch.py # Multi-robot launch file
```

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) for details.
