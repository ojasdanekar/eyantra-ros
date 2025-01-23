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

        # Service clients
        self.spawn_client = self.create_client(Spawn, '/spawn')
        self.coord_client = self.create_client(GetCoordinates, '/get_coordinates')

        # Publisher and subscriber
        self.pose_subscription = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        self.pub1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pub2 = self.create_publisher(Pose,'/turtle1_result',10)

        # Variables
        self.turtle_poses = {'turtle1': Pose()}  # Initialize with turtle1's pose
        self.next_turtle_index = 2
        self.next_turtle = Pose()
        self.following_turtles = []

       
        

        # Get initial coordinates and spawn the first turtle
        self.request_coordinates()

        # Timer for moving turtles
        self.move_turtles_timer = self.create_timer(0.1, self.move_turtles)

  
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
        
            response = future.result()
            self.next_turtle.x = response.x
            self.next_turtle.y = response.y
            self.next_turtle.theta = 0.0
            self.get_logger().info(f'Received coordinates: x={response.x}, y={response.y}')
            self.spawn_turtle(self.next_turtle_index)
        

    def spawn_turtle(self, next_turt):
        

        spawn_req = Spawn.Request(
            x=self.next_turtle.x,
            y=self.next_turtle.y,
            theta=self.next_turtle.theta,
            name=f'turtle{next_turt}'
        )
        future = self.spawn_client.call_async(spawn_req)
        future.add_done_callback(lambda future: self.handle_spawn_response(future, next_turt))

    def handle_spawn_response(self, future, turtle_index):
        
            future.result()
            self.get_logger().info(f'Turtle{turtle_index} spawned successfully.')
            self.turtle_poses[f'turtle{turtle_index}'] = Pose(
                x=self.next_turtle.x, y=self.next_turtle.y, theta=self.next_turtle.theta
            )
            self.create_turtle_subscription(f'turtle{turtle_index}')
        

    def calculate_movement(self, target_pose, current_pose):
        if target_pose is None or current_pose is None:
            return 0.0, 0.0
        distance = math.sqrt((target_pose.x - current_pose.x) ** 2 + (target_pose.y - current_pose.y) ** 2)
        angle = math.atan2(target_pose.y - current_pose.y, target_pose.x - current_pose.x)
        angle_diff = angle - current_pose.theta
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        return distance, angle_diff

    def move_turtles(self):
        

        target_turtle = f'turtle{self.next_turtle_index}'
        target_pose = self.turtle_poses.get(target_turtle)

       
        distance, angle = self.calculate_movement(target_pose, self.turtle_poses['turtle1'])
        twist_msg = Twist()
        linear_gain = 1.0
        angular_gain = 4.0

        if distance < 0.8:
            self.following_turtles.append(f'turtle{self.next_turtle_index}')
            self.next_turtle_index += 1
            self.pub2.publish(self.turtle_poses['turtle1'])
            if self.next_turtle_index <= 11 :
                self.request_coordinates()
        
            
            self.stop_all_pens()
        else:
            twist_msg.linear.x = min(linear_gain * distance, 1.0)
            twist_msg.angular.z = angular_gain * angle
            self.pub1.publish(twist_msg)

            self.follow_turtle_sequence()
            

    def follow_turtle_sequence(self):
        gap_distance = 0.6
        leader_pose = self.turtle_poses['turtle1']

        for follower in self.following_turtles:
            follower_pose = self.turtle_poses.get(follower)
            if not follower_pose:
                continue

            new_x = leader_pose.x - gap_distance * math.cos(leader_pose.theta)
            new_y = leader_pose.y - gap_distance * math.sin(leader_pose.theta)

            teleport_client = self.create_client(TeleportAbsolute, f'/{follower}/teleport_absolute')
            teleport_request = TeleportAbsolute.Request(x=new_x, y=new_y, theta=leader_pose.theta)
            teleport_client.call_async(teleport_request)

            leader_pose = self.turtle_poses[follower]
    
    def stop_all_pens(self):
        # Loop through all the turtles in the following sequence and turn off their pens
        for turtle in self.following_turtles:
            pen_client = self.create_client(SetPen, f'/{turtle}/set_pen')
            pen_request = SetPen.Request(off=True)
            pen_client.call_async(pen_request)


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
