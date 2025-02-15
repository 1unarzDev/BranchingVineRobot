#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import branching_vine_robot.config as config
from interfaces.msg import DepthClustered, Goal
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide" 
import pygame
from enum import Enum
import numpy as np
import random
import cv2

# Constants
FONT_SIZE = 40
FONT = "Segoe UI Medium"
FONT_COLOR = "black"
BACKGROUND_COLOR = "white"
SCREEN_WIDTH = 1280
SCREEN_HEIGHT = 720

screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

def id_to_color(cluster_id, seed=42):
    """
    Maps a cluster ID to a unique RGB color.
    
    Args:
        cluster_id (int): The unique cluster ID.
        seed (int): A fixed seed for color consistency.
    
    Returns:
        tuple: (R, G, B) color.
    """
    random.seed(cluster_id + seed)  # Ensure consistency across frames
    return (random.randint(50, 255), random.randint(50, 255), random.randint(50, 255))

class InteractState(Enum):
    PATH = 0
    PAN = 1
    ROTATE = 2

class Tabs(Enum):
    TWO_DIM = 0
    THREE_DIM = 1 

class GUI(Node):
    def __init__(self):
        super().__init__("gui_node")
        
        self.goal_publisher = self.create_publisher(
            Goal, "/control/goal", 10
        )

        self.cluster_subscriber = self.create_subscription(
            DepthClustered, "/depth/clusters", self.cluster_callback, 10
        )
        
        self.depth_subscriber = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10
        )
        
        self.bridge = CvBridge()
        self.display_timer = self.create_timer(0, self.display)
        self.centroids_x, self.centroids_y = np.array([]), np.array([])

        self.tab = 0
        self.action = InteractState.PAN

        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        self.font = pygame.font.SysFont(FONT, FONT_SIZE)
        self.text = "Test"

    def depth_callback(self, msg):
        # Convert ROS Image message to NumPy array
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Normalize depth image for visualization (convert to 8-bit grayscale)
        depth_image = np.nan_to_num(depth_image)  # Replace NaNs with 0
        depth_image = np.clip(depth_image, 0, 5000)  # Clip depth values to 5m for visualization
        depth_image = (255 * (depth_image / np.max(depth_image))).astype(np.uint8)  # Normalize to 0-255

        # Convert to a Pygame surface
        depth_surface = pygame.surfarray.make_surface(cv2.applyColorMap(cv2.convertScaleAbs(depth_image.T, alpha = 1), cv2.COLORMAP_JET))

        # Display in pygame
        self.screen.blit(pygame.transform.scale(depth_surface, (SCREEN_WIDTH, SCREEN_HEIGHT)), (0, 0))

        # Draw text using a surface
        font_surface = self.font.render(self.text, True, FONT_COLOR)    
        self.screen.blit(font_surface, (10, 10))

        for i in range(len(self.centroids_x)):
            pygame.draw.circle(self.screen, id_to_color(self.labels[i]), (self.centroids_x[i], self.centroids_y[i]), 10)

        pygame.display.flip()


    def cluster_callback(self, msg):
        self.centroids_x = msg.centroid_x
        self.centroids_y = msg.centroid_y
        self.labels = msg.labels

    def display(self):
        # Event handler, TODO: incorporate different functionality for each tab
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
                break

        

        pygame.display.update()

def main(args=None):
    rclpy.init(args=args)
    gui = GUI()
    
    try:
        rclpy.spin(gui)
    except KeyboardInterrupt:
        gui.get_logger().info("GUI shutting down")
    finally:
        gui.destroy_node()
    
if __name__ == "__main__":
    main()