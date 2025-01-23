import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from my_robot_interfaces.action import MoveCircle # Custom action definition


class TurtleCircleActionClient(Node):
    
    """
    A ROS 2 node that acts as an action client to control the turtle's motion 
    in a circle using the MoveCircle action. Feedback is published on the 
    /turtle_circle_feedback topic.
    """

    def __init__(self):
        super().__init__('turtle_circle_action_client')
        self._action_client = ActionClient(self, MoveCircle, 'move_circle')

        # Create a publisher for the /turtle_circle_feedback topic
        self.feedback_publisher = self.create_publisher(String, '/turtle_circle_feedback', 10)

    def send_goal(self, radius, speed):
        """
        Sends a goal to the action server specifying the radius and speed for the turtle's circular motion.

        Args:
        - radius (float): The radius of the circle.
        - speed (float): The speed of the turtle.
        """
        goal_msg = MoveCircle.Goal()
        goal_msg.radius = radius
        goal_msg.speed = speed
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )


    def goal_response_callback(self, future):
        
        """
        Handles the server's response to the goal request (accepted or rejected).

        Args:
        - future: A Future object containing the server's response.
        """
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted. Waiting for result...')
        # Request the result asynchronously and specify a callback for when it's ready
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        
        """
        Processes and logs the feedback from the action server. Also publishes the 
        feedback distance on the /turtle_circle_feedback topic.

        Args:
        - feedback_msg: The feedback message sent by the server.
    
        """
        feedback = feedback_msg.feedback
        combined_message =  f"{feedback.distance_traveled}"
            
            # Create a String message to publish
        msg = String()
        msg.data = combined_message
        
        
        self.feedback_publisher.publish(msg)
        # Extract the feedback content
        
        # Publish the feedback distance on /turtle_circle_feedback, format f"{feedback.distance_traveled}"

    def result_callback(self, future):
        
        """
        Handles the result returned by the action server once the goal is completed.

        Args:
        - future: A Future object containing the server's result.
        """
        
        
        result = future.result().result
        self.get_logger().info(f'{result.total_distance}')
        rclpy.shutdown()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    action_client = TurtleCircleActionClient()
    
    ################## DO NOT MODIFY RADIUS OR SPEED ################
    # Send a goal to the action server with radius=2.0 and speed=2.0
    action_client.send_goal(radius=2.0, speed=2.0)
    #################################################################
    
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
