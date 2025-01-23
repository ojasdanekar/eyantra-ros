import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, SetPen, TeleportAbsolute
from geometry_msgs.msg import Twist
from my_robot_interfaces.srv import GetCoordinates  
import math

class MySpawner(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

    ######################################################## HINT #######################################################

        self.spawn_client = self.create_client(Spawn, '/spawn')
        self.coord_client = self.create_client(GetCoordinates,'/get_coordinates')
        self.pose_subscription = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        self.pub1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) 
        self.turtle_poses = {'turtle1': Pose()}  # Initialize with turtle1's pose
        self.next_turtle_index = 2
        self.next_turtle = Pose()
        
        self.following_turtles = []



        # Create the service client for getting spawn coordinates
        
        


        # Spawn the first turtle and start movement
        self.request_coordinates()
        self.get_logger().info(f'Starting movement towards turtle{self.next_turtle_index}')
        self.move_turtles = self.create_timer(0.1, self.move_turtles) 


    def pose_callback(self, msg):
        self.turtle_poses['turtle1'] = msg

    def create_turtle_subscription(self, turtle_name):
        self.create_subscription(Pose, f'/{turtle_name}/pose', lambda msg: self.update_turtle_pose(turtle_name, msg), 10)

    def update_turtle_pose(self, turtle_name, msg):
        self.turtle_poses[turtle_name] = msg

    def request_coordinates(self):
        coord_req = GetCoordinates.Request()
        future = self.coord_client.call_async(coord_req)
        future.add_done_callback(self.handle_coords)

    def handle_coords(self, future):
        try:
            response = future.result()
            self.next_turtle.x = response.x
            self.next_turtle.y = response.y
            self.next_turtle.theta = 0.0
            self.get_logger().info(f'Received coordinates: x={response.x}, y={response.y}')
            self.spawn_turtle(self.next_turtle_index)
        except Exception as e:
            self.get_logger().error(f'Error receiving coordinates: {e}')

    
 
    def spawn_turtle(self, next_turt):
        if next_turt > 10:
            self.get_logger().info('Maximum number of turtles (10) reached. No more spawning.')
            return

        spawn_req = Spawn.Request(
            x=self.next_turtle.x,
            y=self.next_turtle.y,
            theta=self.next_turtle.theta,
            name=f'turtle{next_turt}'
        )
        future = self.spawn_client.call_async(spawn_req)
        future.add_done_callback(lambda future: self.handle_spawn_response(future, next_turt))

    def handle_spawn_response(self, future, turtle_index):
        try:
            future.result()
            self.get_logger().info(f'Turtle{turtle_index} spawned successfully.')
            self.turtle_poses[f'turtle{turtle_index}'] = Pose(
                    x=self.next_turtle.x, y=self.next_turtle.y, theta=self.next_turtle.theta
                )
            self.create_turtle_subscription(f'turtle{turtle_index}')
        except Exception as e:
            self.get_logger().error(f'Failed to spawn turtle: {e}')


    
    def calculate_movement(self, target_pose, current_pose):
        if target_pose is None or current_pose is None:
            return

            # Calculate the distance and angle to the goal
        current_x, current_y = current_pose.x, current_pose.y
        goal_x, goal_y = target_pose.x, target_pose.y
            
            # Calculate the distance to the goal
        distance = math.sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2)
            
            # Calculate the angle to the goal
        angle = math.atan2(goal_y - current_y, goal_x - current_x)
            
            # Normalize the angle to be between [-pi, pi]
        angle_diff = angle- current_pose.theta
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi
            return distance, angle_diff
    
    
    

        
    
    def move_turtles(self):
    
            # Determine the next target turtle (based on next_turtle_index)
        target_turtle = f'turtle{self.next_turtle_index}'
        target_pose = self.turtle_poses.get(target_turtle)

            # If target turtle pose is not available, try to spawn it
        if not target_pose:
            self.get_logger().info(f'Target pose for {target_turtle} not available. Attempting to spawn it.')
            self.request_coordinates()
            return
            
            # Calculate the distance and angle needed to move towards the target turtle's pose
        distance, angle = self.calculate_movement(target_pose, self.turtle_poses['turtle1'])
        twist_msg1 = Twist()
        linear_gain = 1.0  # Forward speed gain
        angular_gain = 4.0

            # If the distance to the target turtle is less than 0.8 units, stop moving turtle1
        if distance < 0.8:
                self.following_turtles.append(f'turtle{self.next_turtle_index}')
                self.next_turtle_index += 1
                self.stop_all_pens()
    
                if self.next_turtle_index <= 10:
                    self.get_logger().info(f'Spawning next turtle: {self.next_turtle_index}')
                    self.spawn_turtle(self.next_turtle_index)
                else:
                    self.get_logger().info('Maximum number of turtles (10) reached.')
        else:
                twist_msg1.linear.x = min(linear_gain * distance, 1.0)  # Limit forward speed
                twist_msg1.angular.z = angular_gain * angle
                self.pub1.publish(twist_msg1)
                self.follow_turtle_sequence()
                

    def follow_turtle_sequence(self):
        
            # Define a gap distance for following turtles to maintain from turtle1
            gap_distance = 0.6
            leader_pose = self.turtle_poses['turtle1']


            # Loop through each turtle that is following turtle1
            for follower in self.following_turtles:
                # Skip the follower if its pose is not yet available
                if not self.turtle_poses[follower]:
                    continue
                
                # Calculate the new position for the follower to maintain the gap distance from the leader
                new_x = leader_pose.x - gap_distance * math.cos(leader_pose.theta)
                new_y = leader_pose.y - gap_distance * math.sin(leader_pose.theta)

                # Create a client for teleporting the follower to its new position
                teleport_client = self.create_client(TeleportAbsolute, f'/{follower}/teleport_absolute')
                teleport_request = TeleportAbsolute.Request(x=new_x, y=new_y, theta=leader_pose.theta)
                
                # Call the teleportation service to move the follower to the new position
                teleport_client.call_async(teleport_request)
            
                # Update the leader's position to the follower's position for the next iteration
                leader_pose = self.turtle_poses[follower]

    def stop_all_pens(self):
            # Loop through all the turtles in the following sequence and turn off their pens
        for turtle in self.following_turtles:
            pen_client = self.create_client(SetPen, f'/{turtle}/set_pen')
            pen_request = SetPen.Request(off=True)
            pen_client.call_async(pen_request)


    
    ##################################################################################################################################

def main(args=None):
    rclpy.init(args=args)
    node = MySpawner('pose_listener')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
