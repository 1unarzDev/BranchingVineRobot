#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from typing import Enum

class States(Enum):
    ...

class StateMachine(Node):
    def __init__(self):
        super().__init__('state_node') 
      
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

def main(args=None):
    rclpy.init(args=args)
    state = StateMachine()
    
    try:
        rclpy.spin(state)
    except KeyboardInterrupt:
        state.get_logger().info("Shutting down")
    finally:
        state.destroy_node()
    
if __name__ == "__main__":
    main()