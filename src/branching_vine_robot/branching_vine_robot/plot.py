#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

class Plot(Node):
    def __init__(self):
        super().__init__("plot_node")
        self.display_timer = self.create_timer(0, self.display_loop)

        # Create the figure and axes object
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        # Generate random 3D data points
        num_points = 50
        x = np.random.rand(num_points)
        y = np.random.rand(num_points)
        z = np.random.rand(num_points)
        
        # Create the scatter plot
        ax.scatter(x, y, z, c='r', marker='o')
        
        # Set labels for the axes
        ax.set_xlabel('X Axis')
        ax.set_ylabel('Y Axis')
        ax.set_zlabel('Z Axis')
        
        # Set title
        ax.set_title('3D Scatter Plot of Points')
        
        # Display the plot
        plt.show()
        
    def display_loop(self):
        ...

class Branch():
    def __init__(self, basePos):
        self.base = basePos

class Vec3():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        
    def __add__(self, o):
        return Vec3(self.x + o.x, self.y + o.y, self.z + o.z)

    def __mul__(self, o):
        return
    
    def mdpt(self, o):
        return Vec3((self.x + o.x) / 2, (self.y + o.y) / 2, (self.z + o.z) / 2)

def main(args=None):
    rclpy.init(args=args)
    plot = Plot()
    
    try:
        rclpy.spin(plot)
    except KeyboardInterrupt:
        plot.get_logger().info("Shutting down")
    finally:
        plot.destroy_node()
    
if __name__ == "__main__":
    main()