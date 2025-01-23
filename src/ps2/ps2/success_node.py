import rclpy
from rclpy.node import Node

class HelloWorld(Node):
    def __init__(self):
        super().__init__('success_node')
        self.get_logger().info('Package created successfully')

def main(args=None):
    rclpy.init(args=args)
    node = HelloWorld()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()