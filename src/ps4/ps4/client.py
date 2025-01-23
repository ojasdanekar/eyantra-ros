import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import PS4Service  
from std_msgs.msg import String

class PS4Client(Node):
    def __init__(self):
        super().__init__('ps4_client')
        
        # Create a client for the PS4 service
        self.sercli = self.create_client(PS4Service, 'ps4_service')

        ####################### Create a Publisher ###########################
        # Create a publisher to publish the response on topic "ps4_result"
        self.publisher = self.create_publisher(String, 'ps4_result', 10)
        #######################################################################

        # Wait for the service to be available
        while not self.sercli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        # Call the service
        self.request_ps4_service()

    ############## Complete This Section ############## 
    def request_ps4_service(self):
        # Create the service request
        request = PS4Service.Request()
        
        # Call the service asynchronously
        future = self.sercli.call_async(request)
        future.add_done_callback(self.handle_response)
        
    
    def handle_response(self, future):
        # This callback is called when the service responds
        response = future.result()
        if response:
            # Combine the message and unique_id into one string
            combined_message = f"{response.message} unique ID: {response.unique_id}"
            
            # Create a String message to publish
            msg = String()
            msg.data = combined_message
            
            # Publish the combined message
            self.publisher.publish(msg)
            self.get_logger().info(f'Published: {msg.data}')
        else:
            self.get_logger().info("Failed to receive a valid response.")
    ###################################################

def main(args=None):
    rclpy.init(args=args)
    client = PS4Client()
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
