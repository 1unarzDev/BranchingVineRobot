import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide" 
import pygame
from enum import Enum

# Constants
FONT_SIZE = 40
FONT = "Segoe UI Medium"
FONT_COLOR = "black"
BACKGROUND_COLOR = "white"
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600

screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

class InteractState(Enum):
    PATH = 0
    PAN = 1
    ROTATE = 2

class Tabs(Enum):
    TWO_DIM = 0
    THREE_DIM = 1 

class GUI(Node):
    def __init__(self):
        super.__init__("vine_robot_gui")
        
        pygame.init()
        
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        self.font = pygame.font.SysFont(FONT, FONT_SIZE)
        self.text = ""
        
        self.display_timer = self.create_timer(0, self.display_loop)
        self.tab = 0
        self.run = True
        self.action = InteractState.PAN
        
        fig = plt.figure()
 
        # syntax for 3-D projection
        ax = plt.axes(projection ='3d')
         
        # defining axes
        z = np.linspace(0, 1, 100)
        x = z * np.sin(25 * z)
        y = z * np.cos(25 * z)
        c = x + y
        ax.scatter(x, y, z, c = c)
         
        # syntax for plotting
        ax.set_title('3d Scatter plot geeks for geeks')
        plt.show()
        
    def display_loop(self):
        if not self.run:
            return
        
        # Event handler, TODO: incorporate different functionality for each tab
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_2:
                    self.tab = 2
            elif event.type == pygame.MOUSEBUTTONUP:
                pos = pygame.mouse.get_pos()
            elif event.type == pygame.QUIT:
                self.quit()
                self.run = False
                return

        # Clear canvas for new drawing
        self.screen.fill(BACKGROUND_COLOR)
        
        # Draw text using a surface
        font_surface = self.text.render(self.words, True, FONT_COLOR)    
        self.screen.blit(font_surface, (10, 10))

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
        gui.get_logger().info("Shutting down")
    finally:
        gui.destroy_node()
        rclpy.shutdown()()
    
if __name__ == "__main__":
    main()