import branching_vine_robot.utils.controller
import branching_vine_robot.utils.kinematics
import rclpy
from rclpy.node import Node

class PneumaticsNode(Node):
    def __init__(self):
        super().__init__('pneumatics_node')

def main(args=None):
    rclpy.init(args=args)
    pneumatics_node = PneumaticsNode()

    try:
        rclpy.spin(pneumatics_node)
    except KeyboardInterrupt:
        pneumatics_node.get_logger().info("Shutting down")
    finally:
        pneumatics_node.destroy_node()

if __name__ == "__main__":
    main()