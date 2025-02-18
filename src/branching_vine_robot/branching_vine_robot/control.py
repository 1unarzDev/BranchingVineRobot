#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interfaces.msg import Goal
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge

from branching_vine_robot.config import LOOKAHEAD_DIST
from branching_vine_robot.utils.kinematics import calc_actuator

import serial
import numpy as np

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

# Takes in an angle or speed depending on the motor type and sends the command to the mcu
def send_motor_command(motor, speed):
    command = f"{motor} {speed}\n"
    ser.write(command.encode())  # Send command
    response = ser.readline().decode().strip()  # Read response
    return f"Arduino: {response}"

def stop_all_motors():
    for i in range(16):
        send_motor_command(i, 0)

class Control(Node):
    def __init__(self):
        super().__init__("cluster_node")
    
        self.goal_subscriber = self.create_subscription(
            Goal, "control/goals", self.goal_callback, 10
        )
        
        self.info_subscriber = self.create_subscription(
            CameraInfo, 'camera/camera/depth/camera_info', self.camera_info_callback, 10
        )

        self.fx = self.fy = self.cx = self.cy = None  # Camera intrinsics
        self.bridge = CvBridge()
        
        self.goal_x = 0
        self.goal_y = 0

        stop_all_motors()

    def camera_info_callback(self, msg):
        """ Get camera intrinsics from CameraInfo topic. """
        self.fx = msg.k[0]  # Focal length in x
        self.fy = msg.k[4]  # Focal length in y
        self.cx = msg.k[2]  # Optical center x
        self.cy = msg.k[5]  # Optical center y
        
    def img_2_sphere(self, x, y):
        """ Convert a single point from image coordinates to spherical coordinates (theta, phi). """
        if self.fx is None:
            self.get_logger().warn("Camera info not received yet")
            return

        # Compute theta and phi using camera intrinsics
        self.goal_theta = np.arctan2(x - self.cx, self.fx)
        self.goal_phi = np.arctan2(y - self.cy, self.fy)

def main(args=None):
    rclpy.init(args=args)
    control_node = Control()
    
    try:
        rclpy.spin(control_node)
    except KeyboardInterrupt:
        control_node.get_logger().info("State machine node shutting down")
    finally:
        ser.close()
        control_node.destroy_node()
    
if __name__ == "__main__":
    main()