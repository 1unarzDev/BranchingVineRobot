#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class State(Node):
    def __init__(self):
        super().__init__('branching_vine_robots') 
      
        self.states = {
            "manual": self.manual_state,
            "stop": self.stop_state,
            "move": self.move_state,
            "branch": self.branch_state,
        }
    
    def manual_state():
        return
    
    def stop_state():
        return
    
    def move_state():
        return

    def branch_state():
        return    

def main():
    rclpy.init()
    
if __name__ == "__main__":
    main()