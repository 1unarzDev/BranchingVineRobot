from enum import Enum

class States(Enum):
    MANUAL = 0
    STOP = 1
    MOVE = 2
    BRANCH = 3

class StateMachine():
    def __init__(self):
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