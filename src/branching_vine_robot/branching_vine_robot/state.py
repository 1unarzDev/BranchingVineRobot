#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from enum import Enum
import branching_vine_robot.pneumatics

class States(Enum):
    MANUAL = 0
    STOP = 1
    MOVE = 2
    BRANCH = 3

class StateMachine(Node):
    def __init__(self):
        super().__init__('state_node') 

        self.state_timer = self.create_timer(0, self.state_callback)
      
        self.state_map = {
            States.MANUAL: self.manual_state,
            States.STOP: self.stop_state,
            States.MOVE: self.move_state,
            States.BRANCH: self.branch_state,
        }

        self.state = States.MANUAL
    
    def manual_state(self):
        ...
        # self.get_logger().info("Starting manual mode...")
    
    def stop_state(self):
        ...
        # self.get_logger().info("Stopping robot...")
    
    def move_state(self):
        ...
        # self.get_logger().info("Moving robot...")

    def branch_state(self):
        ...
        # self.get_logger().info("Robot branching...")
    
    def state_callback(self):
        ...
        # self.state_map[self.state]()

def main(args=None):
    rclpy.init(args=args)
    state = StateMachine()
    
    try:
        rclpy.spin(state)
    except KeyboardInterrupt:
        state.get_logger().info("State machine node shutting down")
    finally:
        state.destroy_node()
    
if __name__ == "__main__":
    main()