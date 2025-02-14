#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import branching_vine_robot.config as config
from interfaces.msg import DepthClustered, Goal

import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide" 
import pygame
from enum import Enum
import numpy as np
import random

# Constants
FONT_SIZE = 40
FONT = "Segoe UI Medium"
FONT_COLOR = "black"
BACKGROUND_COLOR = "white"
SCREEN_WIDTH = config.RGB_WIDTH
SCREEN_HEIGHT = config.RGB_HEIGHT

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


def depth_to_color(depth, min_depth, max_depth):
    """
    Maps depth values to a blue-to-red gradient.
    
    Args:
        depth (float): The depth value.
        min_depth (float): Minimum depth in range.
        max_depth (float): Maximum depth in range.
    
    Returns:
        tuple: (R, G, B) color mapped to depth.
    """
    # Normalize depth to range [0, 1]
    normalized = (depth - min_depth) / (max_depth - min_depth)
    normalized = max(0, min(1, normalized))  # Clamp between 0 and 1

    # Convert to RGB using a blue-to-red colormap
    r = int(255 * normalized)         # Red increases with depth
    g = 0
    b = int(255 * (1 - normalized))   # Blue decreases with depth

    return (r, g, b)

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
        
        self.depth_subscriber = self.create_subscription(
            DepthClustered, "/depth/clusters", self.depth_callback, 10
        )
        
        self.display_timer = self.create_timer(0, self.display)
        
        self.depths = np.array([])
        self.labels = np.array([])

        self.tab = 0
        self.action = InteractState.PAN

        pygame.init()
        
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        self.font = pygame.font.SysFont(FONT, FONT_SIZE)
        self.text = ""

        self.depths, self.labels, self.centroids_x, self.centroids_y = [], [], [], []
        
    def depth_callback(self, msg):
        self.depths = np.array(msg.depths).reshape(msg.height, msg.width)
        self.labels = np.array(msg.labels).reshape(msg.height, msg.width)
        self.centroids_x = msg.centroid_x
        self.centroids_y = msg.centroid_y
        
    def display(self):
        # Event handler, TODO: incorporate different functionality for each tab
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_2:
                    self.tab = 2
            elif event.type == pygame.MOUSEBUTTONUP:
                pos = pygame.mouse.get_pos()
                
                goal_msg = Goal(x=pos[0], y=pos[1])

                self.goal_publisher().publish(goal_msg)
            elif event.type == pygame.QUIT:
                pygame.quit()
                break

        # Clear canvas for new drawing
        self.screen.fill(BACKGROUND_COLOR)

        for i in range(len(self.depths)):
            for j in range(len(self.depths[i])):
                    pygame.draw.circle(self.screen, depth_to_color(self.depths[i][j], 0, 2), (j, i), 1)
                
        for i in range(len(self.centroids_x)):
            pygame.draw.circle(self.screen, (0, 0, 0), (self.centroids_x[i], self.centroids_y[i]), 2)
        
        # Draw text using a surface
        font_surface = self.font.render(self.text, True, FONT_COLOR)    
        self.screen.blit(font_surface, (10, 10))
        
        pygame.display.update()

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
    gui = GUI()
    
    try:
        rclpy.spin(gui)
    except KeyboardInterrupt:
        gui.get_logger().info("GUI shutting down")
    finally:
        gui.destroy_node()
    
if __name__ == "__main__":
    main()