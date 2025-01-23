import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose  # This is still valid if 'pose' is using Pose
from turtlesim.srv import SetPen
from std_msgs.msg import Bool
from math import sqrt, atan2, pi

class GetPointsSubscriber(Node):
    def __init__(self):
        super().__init__('get_points_subscriber')
        
        ########################## DO NOT MODIFY THIS LINE ###########################
        # Subscribe to the 'goal_point' topic to receive goal points
        self.subscription = self.create_subscription(Point, 'goal_point', self.point_callback, 10)
        ##############################################################################

        ########################### FILL IN THE CODE FOR THE MENTIONED COMMENT####################

        # Subscribe to the 'turtle1/pose' topic instead of '/turtle1/pose'
        # Assuming the goal status is of type 'Pose' (you might need to change this based on the actual message type)
        self.pose_subscription = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        
        # Write a publisher for topic '/turtle1/cmd_vel' message type is 'Twist'
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_publisher = self.create_publisher(Pose,'/turtle1/goal_status',10)
        ##########################################################################################

        ########################## DO NOT MODIFY THESE LINES ###########################
        self.goal_reached_publisher = self.create_publisher(Bool, 'goal_reached', 10)
        
        # Client for the SetPen service, to move the pen up and down
        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        while not self.set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for SetPen service...')

        # Timer to periodically move the turtle
        self.move_to_goal = self.create_timer(0.1, self.move_to_goal)    
        
        self.goal = None  # Initialize goal as None to avoid AttributeError
        self.goal_in_progress = False  # Flag to track if the turtle is moving towards a goal
        self.current_pose = None  # Initial pose of the turtle
        self.pen_down = False  # Flag to track whether the pen is down or not 
        ################################################################################


    

    
     
    def pose_callback(self, msg):
        """Callback function to update the turtle's current position."""
        # Save the current position of the turtle from the Pose message
        self.current_pose = msg

        

    def point_callback(self, point):
        """Callback function to receive a new goal point."""

        self.goal = point
        self.goal_in_progress = True
        self.get_logger().info(f'Received new goal: ({point.x}, {point.y}, {point.z})')
        
    ##################################### WRITE THE LOGIC ###############################
    def move_to_goal(self):
        """Function to move the turtle towards the current goal."""
        if self.goal is None or self.current_pose is None:
            return

        # Calculate the distance and angle to the goal
        current_x, current_y = self.current_pose.x, self.current_pose.y
        goal_x, goal_y = self.goal.x, self.goal.y
        
        # Calculate the distance to the goal
        distance = sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2)
        
        # Calculate the angle to the goal
        angle_to_goal = atan2(goal_y - current_y, goal_x - current_x)
        
        # Normalize the angle to be between [-pi, pi]
        angle_diff = angle_to_goal - self.current_pose.theta
        if angle_diff > pi:
            angle_diff -= 2 * pi
        elif angle_diff < -pi:
            angle_diff += 2 * pi

        # Calculate velocity commands
        linear_gain = 1.0  # Forward speed gain
        angular_gain = 4.0  # Turning speed gain

        twist = Twist()
        twist.linear.x = min(linear_gain * distance, 1.0)  # Limit forward speed
        twist.angular.z = angular_gain * angle_diff

        # Publish velocity commands to move the turtle
        self.velocity_publisher.publish(twist)
        

        # If the turtle is close to the goal, stop the movement
        if distance < 0.1:  # Threshold for reaching the goal
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.velocity_publisher.publish(twist)
            self.pose_publisher.publish(self.current_pose)
            
            ########################## DO NOT MODIFY THESE LINES ###########################
            self.get_logger().info('Reached the goal!')
    
            self.goal_in_progress = False  # Reset the flag when goal is reached

            # Publish that the goal is reached
            self.goal_reached_publisher.publish(Bool(data=True))

            # Set pen down if not already down
            if not self.pen_down:
                self.set_pen(True)  # Call method to lower the pen
                self.pen_down = True  # Update flag to prevent repeated pen-down calls
            ################################################################################

    ########################## DO NOT MODIFY THESE LINES ###########################
    def set_pen(self, pen_down):
        """Method to set pen state: down if pen_down is True, otherwise up."""
        request = SetPen.Request()
        request.r = 255
        request.g = 255
        request.b = 255
        request.width = 2
        request.off = not pen_down 
        self.set_pen_client.call_async(request)
        self.get_logger().info('Pen set down.' if pen_down else 'Pen set up.')
    #################################################################################
    
def main(args=None):
    rclpy.init(args=args)
    subscriber = GetPointsSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

