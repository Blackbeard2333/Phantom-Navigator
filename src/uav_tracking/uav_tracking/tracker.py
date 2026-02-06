import rclpy
from rclpy.node import Node

class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')
        self.get_logger().info('Object Tracker Node has started.')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTracker()
    rclpy.spin(node)
    rclpy.shutdown()
