################################################    DO NOT MODIFY    #########################################################

from random import uniform
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import GetCoordinates
from std_msgs.msg import String

class CoordinateServer(Node):
    def __init__(self):
        super().__init__('coordinate_server')
        self.srv = self.create_service(GetCoordinates, 'get_coordinates', self.generate_coordinates)
        self.publisher = self.create_publisher(String, 'coordinates', 10)  
        self.get_logger().info('Coordinate server ready.')

    def generate_coordinates(self, request, response):
        response.x = uniform(1.0, 10.0)
        response.y = uniform(1.0, 10.0)
        
        # Log and publish the coordinates
        self.get_logger().info(f'Responding with coordinates: x={response.x}, y={response.y}')
        self.publish_coordinates(response.x, response.y)
        
        return response

    def publish_coordinates(self, x, y):
        msg = String()
        msg.data = f"Coordinates: x={x}, y={y}"  
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    server = CoordinateServer()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    
################################################################################################################################
