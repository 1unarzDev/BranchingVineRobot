#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interfaces.msg import Clusters, Goal
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from branching_vine_robot.config import MAX_DEPTH, MIN_DEPTH, DEPTH_HEIGHT, DEPTH_WIDTH

import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide" 
import pygame
from enum import Enum
import numpy as np
import cv2

# Constants
FONT_SIZE = 40
FONT = "Segoe UI Medium"
FONT_COLOR = "black"
BACKGROUND_COLOR = "white"
SCREEN_WIDTH = DEPTH_WIDTH
SCREEN_HEIGHT = DEPTH_HEIGHT

screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

class InteractState(Enum):
    PLOT = 0

class Tabs(Enum):
    PATH = 0

def depth_to_color(depth, min_depth, max_depth):
    """
    Maps a depth value to an RGB color where:
    - Min depth (close) → Green (0, 255, 0)
    - Max depth (far) → Pink (255, 105, 180)
    
    Parameters:
    - depth: float, the depth value to map
    - min_depth: float, the minimum depth value (close range)
    - max_depth: float, the maximum depth value (far range)
    
    Returns:
    - tuple (R, G, B) with values between 0 and 255
    """
    # Normalize depth to range [0, 1]
    norm = np.clip((depth - min_depth) / (max_depth - min_depth), 0, 1)
    
    # Interpolate between Green (0,255,0) and Pink (255,105,180)
    r = int(255 * norm + 0 * (1 - norm))  # From 0 → 255
    g = int(105 * norm + 255 * (1 - norm))  # From 255 → 105
    b = int(180 * norm + 0 * (1 - norm))  # From 0 → 180

    return (r, g, b)

class GUI(Node):
    def __init__(self):
        super().__init__("gui_node")
        
        self.goal_publisher = self.create_publisher(
            Goal, "/control/goal", 10
        )

        self.cluster_subscriber = self.create_subscription(
            Clusters, "/depth/clusters", self.cluster_callback, 10
        )
        
        self.depth_subscriber = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10
        )
        
        self.bridge = CvBridge()
        self.display_timer = self.create_timer(0, self.display)
        self.x, self.y, self.z, self.sizes = np.array([]), np.array([]), np.array([]), np.array([])

        self.tab = Tabs.PATH
        self.action = InteractState.PLOT

        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        self.font = pygame.font.SysFont(FONT, FONT_SIZE)
        self.text = ""

    def depth_callback(self, msg):
        # Convert ROS Image message to NumPy array
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Normalize depth image for visualization (convert to 8-bit grayscale)
        depth_image = np.nan_to_num(depth_image)  # Replace NaNs with 0
        # Change 0 to MIN_DEPTH to show clipping used in clustering algorithm
        depth_image = np.clip(depth_image, 0, MAX_DEPTH)  # Clip depth values to 5m for visualization
        depth_image = (255 * (depth_image / np.max(depth_image))).astype(np.uint8)  # Normalize to 0-255

        # Convert to a Pygame surface
        depth_surface = pygame.surfarray.make_surface(cv2.applyColorMap(cv2.convertScaleAbs(depth_image.T, alpha = 1), cv2.COLORMAP_JET))

        # Display in pygame
        self.screen.blit(pygame.transform.scale(depth_surface, (SCREEN_WIDTH, SCREEN_HEIGHT)), (0, 0))

        # Draw text using a surface
        font_surface = self.font.render(self.text, True, FONT_COLOR)    
        self.screen.blit(font_surface, (10, 10))

        for i in range(len(self.x)):
            pygame.draw.circle(self.screen, depth_to_color(self.z[i], 0, MAX_DEPTH), (self.x[i], self.y[i]), self.sizes[i] / 40)

        pygame.display.flip()

    def cluster_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.sizes = msg.sizes

    def display(self):
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_2:
                    self.tab = 2
            elif event.type == pygame.MOUSEBUTTONUP:
                pos = pygame.mouse.get_pos()
                goal_msg = Goal(x=pos[0], y=pos[1])
                self.goal_publisher.publish(goal_msg)
            elif event.type == pygame.QUIT:
                pygame.quit()
                self.get_logger().info("GUI shutting down")
                self.destroy_node()
                break

def main(args=None):
    rclpy.init(args=args)
    gui = GUI()
    
    try:
        rclpy.spin(gui)
    except KeyboardInterrupt:
        gui.get_logger().info("GUI shutting down")
    finally:
        pygame.quit()
        gui.destroy_node()
    
if __name__ == "__main__":
    main()